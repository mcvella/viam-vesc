"""UART/USB serial transport for VESC COMM protocol."""

from __future__ import annotations

import struct
import time
from typing import Any, Optional

import serial

from .transport import VescStatus, VescTransport


class SerialTransport(VescTransport):
    COMM_GET_VALUES = 0x27
    COMM_SET_DUTY = 5
    COMM_SET_CURRENT = 0x01
    COMM_SET_CURRENT_BRAKE = 0x02
    COMM_SET_RPM = 0x03
    COMM_ALIVE = 0x3A

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        timeout: float = 1.0,
        duty_cycle_format: str = "int",
        debug: bool = True,
        logger: Any = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.duty_cycle_format = duty_cycle_format
        self._debug = debug
        self.logger = logger
        self.serial_port: Optional[serial.Serial] = None

    def open(self) -> None:
        self.serial_port = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        if self.logger:
            self.logger.info(
                f"Connected to VESC on {self.port} at {self.baudrate} baud"
            )

    def close(self) -> None:
        if self.serial_port:
            try:
                self.serial_port.close()
            except Exception:
                pass
            self.serial_port = None

    def test_connection(self) -> bool:
        try:
            if self._send_simple_command(self.COMM_GET_VALUES):
                time.sleep(0.1)
                return self._read_response() is not None
            return False
        except Exception as e:
            if self.logger:
                self.logger.error(f"Connection test failed: {e}")
            return False

    def set_duty(self, duty: float) -> bool:
        if self.duty_cycle_format == "float":
            payload = struct.pack(">f", duty)
        else:
            payload = struct.pack(">i", int(duty * 100000))
        return self._send_packet(self.COMM_SET_DUTY, payload)

    def set_rpm(self, rpm: float) -> bool:
        payload = struct.pack(">i", int(rpm))
        return self._send_simple_command(self.COMM_SET_RPM, payload)

    def set_current(self, current: float) -> bool:
        payload = struct.pack(">f", float(current))
        return self._send_simple_command(self.COMM_SET_CURRENT, payload)

    def set_current_brake(self, current: float) -> bool:
        payload = struct.pack(">f", float(current))
        return self._send_simple_command(self.COMM_SET_CURRENT_BRAKE, payload)

    def ping(self) -> bool:
        if not self._send_simple_command(self.COMM_ALIVE):
            return False
        return self._read_response() is not None

    def get_status(self) -> Optional[VescStatus]:
        if not self._send_simple_command(self.COMM_GET_VALUES):
            return None
        payload = self._read_response()
        if not payload:
            return None
        return VescStatus(raw_hex=payload.hex(), last_update=time.time())

    def clear_buffers(self) -> bool:
        if not self.serial_port or not self.serial_port.is_open:
            return False
        self.serial_port.reset_input_buffer()
        self.serial_port.reset_output_buffer()
        return True

    def _crc16(self, data: bytes) -> int:
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def _pack_payload(self, payload: bytes) -> bytes:
        if len(payload) <= 256:
            header = struct.pack("BB", 2, len(payload))
        else:
            header = struct.pack(
                "BBB", 3, len(payload) >> 8, len(payload) & 0xFF
            )
        crc = self._crc16(payload)
        return header + payload + struct.pack(">H", crc) + b"\x03"

    def _send_packet(self, command_id: int, payload: bytes = b"") -> bool:
        if not self.serial_port or not self.serial_port.is_open:
            if self.logger:
                self.logger.error("Serial port not open")
            return False
        try:
            self.serial_port.reset_input_buffer()
            full_payload = struct.pack("B", command_id) + payload
            packet = self._pack_payload(full_payload)
            if self._debug and self.logger:
                self.logger.debug(
                    f"Sending packet: command_id={command_id}, "
                    f"payload={payload.hex()}, full_packet={packet.hex()}"
                )
            self.serial_port.reset_output_buffer()
            self.serial_port.write(packet)
            self.serial_port.flush()
            time.sleep(0.001)
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to send packet: {e}")
            return False

    def _send_simple_command(self, command_id: int, payload: bytes = b"") -> bool:
        if not self.serial_port or not self.serial_port.is_open:
            if self.logger:
                self.logger.error("Serial port not open")
            return False
        try:
            packet = struct.pack("B", 0x02)
            packet += struct.pack("B", len(payload) + 1)
            packet += struct.pack("B", command_id)
            packet += payload
            crc_data = packet[1:]
            crc = self._crc16(crc_data)
            packet += struct.pack(">H", crc)
            packet += struct.pack("B", 0x03)
            if self._debug and self.logger:
                self.logger.debug(f"Sending packet: {packet.hex()}")
            self.serial_port.write(packet)
            self.serial_port.flush()
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to send command: {e}")
            return False

    def _read_response(self, timeout: float = 0.1) -> Optional[bytes]:
        if not self.serial_port or not self.serial_port.is_open:
            return None
        try:
            original_timeout = self.serial_port.timeout
            self.serial_port.timeout = timeout

            start = self.serial_port.read(1)
            if not start or start[0] != 0x02:
                self.serial_port.timeout = original_timeout
                return None

            length_bytes = self.serial_port.read(1)
            if not length_bytes:
                self.serial_port.timeout = original_timeout
                return None

            length = length_bytes[0]
            payload = self.serial_port.read(length)
            if len(payload) != length:
                self.serial_port.timeout = original_timeout
                return None

            crc_bytes = self.serial_port.read(2)
            if len(crc_bytes) != 2:
                self.serial_port.timeout = original_timeout
                return None

            end = self.serial_port.read(1)
            if not end or end[0] != 0x03:
                self.serial_port.timeout = original_timeout
                return None

            received_crc = struct.unpack(">H", crc_bytes)[0]
            calculated_crc = self._crc16(struct.pack("B", length) + payload)
            if received_crc != calculated_crc:
                if self.logger:
                    self.logger.error(
                        f"CRC mismatch: received {received_crc}, "
                        f"calculated {calculated_crc}"
                    )
                self.serial_port.timeout = original_timeout
                return None

            self.serial_port.timeout = original_timeout
            if self._debug and self.logger:
                self.logger.debug(f"Received response: {payload.hex()}")
            return payload
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to read response: {e}")
            return None
