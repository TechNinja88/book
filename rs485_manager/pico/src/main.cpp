#include <Arduino.h>
#include "RS485.h"

// Pin definitions (override via platformio.ini build_flags)
#ifndef RS485_TX_PIN
#define RS485_TX_PIN   0
#endif
#ifndef RS485_RX_PIN
#define RS485_RX_PIN   1
#endif
#ifndef RS485_DE_RE_PIN
#define RS485_DE_RE_PIN 2
#endif
#ifndef SLAVE_ADDRESS
#define SLAVE_ADDRESS  0x01
#endif
#ifndef RS485_BAUD
#define RS485_BAUD     115200
#endif

// LED pin (Pico onboard LED)
#define LED_PIN        25

RS485Manager rs485(Serial1, RS485_DE_RE_PIN, SLAVE_ADDRESS);

static void handlePacket(const Packet &pkt) {
    switch (pkt.cmd) {

        case CMD_PING: {
            uint8_t resp[] = {SLAVE_ADDRESS, 0x00}; // addr, status OK
            rs485.sendAck(pkt.address == 0xFF ? 0x00 : pkt.address, resp, sizeof(resp));
            break;
        }

        case CMD_READ_ADC: {
            if (pkt.len < 1) { rs485.sendNack(pkt.address); break; }
            uint8_t ch = pkt.data[0];
            uint16_t raw = analogRead(26 + ch); // ADC0=GP26, ADC1=GP27, ADC2=GP28
            uint8_t resp[3] = {SLAVE_ADDRESS, (uint8_t)(raw >> 8), (uint8_t)(raw & 0xFF)};
            rs485.sendAck(pkt.address, resp, sizeof(resp));
            break;
        }

        case CMD_SET_LED: {
            if (pkt.len < 1) { rs485.sendNack(pkt.address); break; }
            digitalWrite(LED_PIN, pkt.data[0] ? HIGH : LOW);
            uint8_t resp[] = {SLAVE_ADDRESS, pkt.data[0]};
            rs485.sendAck(pkt.address, resp, sizeof(resp));
            break;
        }

        case CMD_READ_GPIO: {
            if (pkt.len < 1) { rs485.sendNack(pkt.address); break; }
            uint8_t pin = pkt.data[0];
            pinMode(pin, INPUT_PULLUP);
            uint8_t val = digitalRead(pin);
            uint8_t resp[] = {SLAVE_ADDRESS, pin, val};
            rs485.sendAck(pkt.address, resp, sizeof(resp));
            break;
        }

        case CMD_WRITE_GPIO: {
            if (pkt.len < 2) { rs485.sendNack(pkt.address); break; }
            uint8_t pin = pkt.data[0];
            uint8_t val = pkt.data[1];
            pinMode(pin, OUTPUT);
            digitalWrite(pin, val ? HIGH : LOW);
            uint8_t resp[] = {SLAVE_ADDRESS, pin, val};
            rs485.sendAck(pkt.address, resp, sizeof(resp));
            break;
        }

        default:
            rs485.sendNack(pkt.address);
            break;
    }
}

void setup() {
    Serial.begin(115200);   // USB debug
    Serial1.setTX(RS485_TX_PIN);
    Serial1.setRX(RS485_RX_PIN);
    rs485.begin(RS485_BAUD);

    pinMode(LED_PIN, OUTPUT);
    analogReadResolution(12);

    Serial.printf("Pico RS485 slave ready  addr=0x%02X  baud=%u\n",
                  SLAVE_ADDRESS, RS485_BAUD);
}

void loop() {
    Packet pkt;
    if (rs485.receive(pkt)) {
        handlePacket(pkt);
    }
}
