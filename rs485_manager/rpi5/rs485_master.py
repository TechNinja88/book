"""
RS485 master for Raspberry Pi 5.
Matches the packet protocol in the Pico firmware.

Packet format: [0xAA][ADDR][CMD][LEN][DATA...][CRC8]
"""

import serial
import time
import struct
import logging
from enum import IntEnum
from typing import Optional

logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
log = logging.getLogger("rs485")

FRAME_START = 0xAA
TIMEOUT     = 0.5   # seconds


class Cmd(IntEnum):
    PING        = 0x01
    READ_ADC    = 0x02
    SET_LED     = 0x03
    READ_GPIO   = 0x04
    WRITE_GPIO  = 0x05
    ACK         = 0x80
    NACK        = 0x81


def _crc8(data: bytes) -> int:
    crc = 0x00
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def _build_frame(address: int, cmd: Cmd, data: bytes = b"") -> bytes:
    payload = bytes([address, int(cmd), len(data)]) + data
    crc = _crc8(payload)
    return bytes([FRAME_START]) + payload + bytes([crc])


class RS485Master:
    """
    RS485 master running on Raspberry Pi 5.

    Wiring:
        RPi5 UART TX  → MAX485 DI
        RPi5 UART RX  ← MAX485 RO
        RPi5 GPIO XX  → MAX485 DE + RE (tied together)
        MAX485 A/B    → RS485 bus → Pico MAX485 A/B

    For RPi5 built-in UART use /dev/ttyAMA0 (disable BT in config.txt).
    For a USB-UART adapter use /dev/ttyUSB0.
    DE/RE can be left hard-wired HIGH (transmit) if you use a half-duplex
    adapter that handles direction automatically (e.g. USB RS485 dongle).
    """

    def __init__(self, port: str = "/dev/ttyUSB0", baud: int = 115200,
                 de_re_gpio: Optional[int] = None):
        self._port    = port
        self._baud    = baud
        self._de_re   = de_re_gpio
        self._ser: Optional[serial.Serial] = None
        self._gpio_initialized = False

        if de_re_gpio is not None:
            try:
                import RPi.GPIO as GPIO
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(de_re_gpio, GPIO.OUT, initial=GPIO.LOW)
                self._GPIO = GPIO
                self._gpio_initialized = True
            except ImportError:
                log.warning("RPi.GPIO not installed; DE/RE pin not controlled.")

    def open(self):
        self._ser = serial.Serial(
            self._port, self._baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=TIMEOUT,
        )
        log.info("Opened %s @ %d baud", self._port, self._baud)

    def close(self):
        if self._ser:
            self._ser.close()
        if self._gpio_initialized:
            self._GPIO.cleanup()

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *_):
        self.close()

    # ------------------------------------------------------------------ #
    #  Low-level send / receive                                           #
    # ------------------------------------------------------------------ #

    def _set_tx(self):
        if self._gpio_initialized:
            self._GPIO.output(self._de_re, self._GPIO.HIGH)
            time.sleep(0.00001)

    def _set_rx(self):
        if self._gpio_initialized:
            self._GPIO.output(self._de_re, self._GPIO.LOW)

    def _send_raw(self, frame: bytes):
        self._set_tx()
        self._ser.write(frame)
        self._ser.flush()
        self._set_rx()

    def _recv_packet(self) -> Optional[dict]:
        deadline = time.time() + TIMEOUT
        state    = "START"
        buf      = bytearray()
        data_len = 0

        while time.time() < deadline:
            b = self._ser.read(1)
            if not b:
                continue
            byte = b[0]

            if state == "START":
                if byte == FRAME_START:
                    buf.clear()
                    state = "ADDR"
            elif state == "ADDR":
                buf.append(byte)   # address
                state = "CMD"
            elif state == "CMD":
                buf.append(byte)   # cmd
                state = "LEN"
            elif state == "LEN":
                data_len = byte
                buf.append(byte)
                state = "DATA" if data_len > 0 else "CRC"
            elif state == "DATA":
                buf.append(byte)
                if len(buf) == 3 + data_len:
                    state = "CRC"
            elif state == "CRC":
                expected = _crc8(buf)
                if byte != expected:
                    log.warning("CRC mismatch: got 0x%02X expected 0x%02X", byte, expected)
                    state = "START"
                    buf.clear()
                    continue
                return {
                    "address": buf[0],
                    "cmd":     Cmd(buf[1]),
                    "data":    bytes(buf[3:]),
                }
        log.warning("Receive timeout")
        return None

    def _transact(self, address: int, cmd: Cmd, data: bytes = b"") -> Optional[dict]:
        frame = _build_frame(address, cmd, data)
        log.debug("TX addr=0x%02X cmd=%s len=%d", address, cmd.name, len(data))
        self._send_raw(frame)
        pkt = self._recv_packet()
        if pkt is None:
            return None
        if pkt["cmd"] == Cmd.NACK:
            log.warning("NACK from 0x%02X", address)
            return None
        return pkt

    # ------------------------------------------------------------------ #
    #  High-level API                                                     #
    # ------------------------------------------------------------------ #

    def ping(self, address: int) -> bool:
        pkt = self._transact(address, Cmd.PING)
        if pkt:
            log.info("Ping 0x%02X OK", address)
            return True
        return False

    def read_adc(self, address: int, channel: int) -> Optional[int]:
        """Returns raw 12-bit ADC value (0-4095) or None on error."""
        pkt = self._transact(address, Cmd.READ_ADC, bytes([channel]))
        if pkt and len(pkt["data"]) >= 3:
            raw = (pkt["data"][1] << 8) | pkt["data"][2]
            voltage = raw * 3.3 / 4095
            log.info("ADC ch%d  raw=%d  %.3fV", channel, raw, voltage)
            return raw
        return None

    def set_led(self, address: int, state: bool) -> bool:
        pkt = self._transact(address, Cmd.SET_LED, bytes([1 if state else 0]))
        return pkt is not None

    def read_gpio(self, address: int, pin: int) -> Optional[int]:
        """Returns 0 or 1, or None on error."""
        pkt = self._transact(address, Cmd.READ_GPIO, bytes([pin]))
        if pkt and len(pkt["data"]) >= 3:
            return pkt["data"][2]
        return None

    def write_gpio(self, address: int, pin: int, value: int) -> bool:
        pkt = self._transact(address, Cmd.WRITE_GPIO, bytes([pin, value]))
        return pkt is not None

    def scan(self, start: int = 0x01, end: int = 0x7F) -> list[int]:
        """Scan for responsive slave addresses."""
        found = []
        for addr in range(start, end + 1):
            if self.ping(addr):
                found.append(addr)
        return found


# --------------------------------------------------------------------------- #
# Demo / quick test                                                             #
# --------------------------------------------------------------------------- #

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description="RS485 master demo")
    parser.add_argument("--port",    default="/dev/ttyUSB0")
    parser.add_argument("--baud",    default=115200, type=int)
    parser.add_argument("--address", default=0x01,   type=lambda x: int(x, 0))
    parser.add_argument("--de-re",   default=None,   type=int,
                        help="BCM GPIO pin for DE/RE control (optional)")
    args = parser.parse_args()

    with RS485Master(port=args.port, baud=args.baud, de_re_gpio=args.de_re) as m:
        print(f"\n--- Pinging slave 0x{args.address:02X} ---")
        if not m.ping(args.address):
            print("No response. Check wiring and slave address.")
        else:
            print("\n--- ADC channels ---")
            for ch in range(3):
                raw = m.read_adc(args.address, ch)
                if raw is not None:
                    print(f"  CH{ch}: {raw}  ({raw * 3.3 / 4095:.3f} V)")

            print("\n--- LED blink ---")
            for state in [True, False, True, False]:
                m.set_led(args.address, state)
                time.sleep(0.3)

            print("\nDone.")
