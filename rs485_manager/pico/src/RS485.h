#pragma once
#include <Arduino.h>

// Packet format: [0xAA][ADDR][CMD][LEN][DATA x LEN][CRC8]
#define FRAME_START   0xAA
#define MAX_DATA_LEN  64

enum Command : uint8_t {
    CMD_PING        = 0x01,
    CMD_READ_ADC    = 0x02,
    CMD_SET_LED     = 0x03,
    CMD_READ_GPIO   = 0x04,
    CMD_WRITE_GPIO  = 0x05,
    CMD_ACK         = 0x80,
    CMD_NACK        = 0x81,
};

struct Packet {
    uint8_t address;
    Command cmd;
    uint8_t len;
    uint8_t data[MAX_DATA_LEN];
    uint8_t crc;
};

static uint8_t crc8(const uint8_t *buf, uint8_t len) {
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
    }
    return crc;
}

class RS485Manager {
public:
    RS485Manager(HardwareSerial &serial, uint8_t deRePin, uint8_t address)
        : _serial(serial), _deRePin(deRePin), _address(address) {}

    void begin(uint32_t baud) {
        _serial.begin(baud);
        pinMode(_deRePin, OUTPUT);
        _setReceive();
    }

    // Returns true and fills pkt if a valid addressed packet is received.
    bool receive(Packet &pkt) {
        while (_serial.available()) {
            uint8_t b = _serial.read();
            switch (_state) {
                case WAIT_START:
                    if (b == FRAME_START) _state = WAIT_ADDR;
                    break;
                case WAIT_ADDR:
                    _buf[0] = b;
                    _state = (b == _address || b == 0xFF) ? WAIT_CMD : WAIT_START;
                    break;
                case WAIT_CMD:
                    _buf[1] = b;
                    _state = WAIT_LEN;
                    break;
                case WAIT_LEN:
                    _buf[2] = b;
                    _rxIdx = 0;
                    _state = (b > MAX_DATA_LEN) ? WAIT_START : (b > 0 ? WAIT_DATA : WAIT_CRC);
                    break;
                case WAIT_DATA:
                    _buf[3 + _rxIdx++] = b;
                    if (_rxIdx >= _buf[2]) _state = WAIT_CRC;
                    break;
                case WAIT_CRC: {
                    uint8_t expected = crc8(_buf, 3 + _buf[2]);
                    _state = WAIT_START;
                    if (b != expected) break;
                    pkt.address = _buf[0];
                    pkt.cmd     = (Command)_buf[1];
                    pkt.len     = _buf[2];
                    memcpy(pkt.data, _buf + 3, pkt.len);
                    pkt.crc     = b;
                    return true;
                }
            }
        }
        return false;
    }

    void send(uint8_t address, Command cmd, const uint8_t *data, uint8_t len) {
        uint8_t frame[4 + MAX_DATA_LEN + 1];
        frame[0] = FRAME_START;
        frame[1] = address;
        frame[2] = (uint8_t)cmd;
        frame[3] = len;
        if (len && data) memcpy(frame + 4, data, len);
        frame[4 + len] = crc8(frame + 1, 3 + len);

        _setTransmit();
        _serial.write(frame, 5 + len);
        _serial.flush();
        _setReceive();
    }

    void sendAck(uint8_t address, const uint8_t *data = nullptr, uint8_t len = 0) {
        send(address, CMD_ACK, data, len);
    }

    void sendNack(uint8_t address) {
        send(address, CMD_NACK, nullptr, 0);
    }

private:
    HardwareSerial &_serial;
    uint8_t _deRePin;
    uint8_t _address;

    enum State : uint8_t { WAIT_START, WAIT_ADDR, WAIT_CMD, WAIT_LEN, WAIT_DATA, WAIT_CRC };
    State _state = WAIT_START;
    uint8_t _buf[4 + MAX_DATA_LEN];
    uint8_t _rxIdx = 0;

    void _setTransmit() { digitalWrite(_deRePin, HIGH); delayMicroseconds(10); }
    void _setReceive()  { digitalWrite(_deRePin, LOW); }
};
