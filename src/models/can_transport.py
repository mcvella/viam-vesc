"""Linux SocketCAN transport for VESC CAN protocol."""

from __future__ import annotations

import struct
import threading
import time
from typing import Any, Optional

import can

from .transport import VescStatus, VescTransport

CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_CURRENT_BRAKE = 2
CAN_PACKET_SET_RPM = 3
CAN_PACKET_STATUS = 9
CAN_PACKET_STATUS_2 = 14
CAN_PACKET_STATUS_3 = 15
CAN_PACKET_STATUS_4 = 16
CAN_PACKET_PING = 17
CAN_PACKET_PONG = 18
CAN_PACKET_STATUS_5 = 27


class CanTransport(VescTransport):
    def __init__(
        self,
        interface: str,
        vesc_id: int,
        debug: bool = True,
        logger: Any = None,
    ):
        self.interface = interface
        self.vesc_id = vesc_id
        self._debug = debug
        self.logger = logger
        self.bus: Optional[can.BusABC] = None
        self._status = VescStatus()
        self._status_lock = threading.Lock()
        self._stop_listen = threading.Event()
        self._listen_thread: Optional[threading.Thread] = None

    def open(self) -> None:
        try:
            import can.interfaces.socketcan  # noqa: F401
        except ImportError as e:
            raise RuntimeError(
                "SocketCAN backend is unavailable. On a registry/cloud-built "
                "module this usually means python-can was not fully packaged; "
                "rebuild with --collect-all can. For a local module, ensure "
                "python-can is installed in the venv (re-run setup.sh) and you "
                "are on Linux."
            ) from e

        # Filter for extended frames from this VESC ID (any command in bits 15-8).
        # Mask 0xFF matches the low 8 bits (controller id).
        self.bus = can.Bus(
            channel=self.interface,
            interface="socketcan",
            can_filters=[
                {
                    "can_id": self.vesc_id,
                    "can_mask": 0xFF,
                    "extended": True,
                }
            ],
        )
        self._stop_listen.clear()
        self._listen_thread = threading.Thread(
            target=self._listen_for_messages,
            name=f"vesc-can-{self.vesc_id}",
            daemon=True,
        )
        self._listen_thread.start()
        if self.logger:
            self.logger.info(
                f"Connected to VESC on CAN {self.interface} id={self.vesc_id}"
            )

    def close(self) -> None:
        # Signal stop first, then shutdown the bus so recv() unblocks.
        # Keep a local bus reference in the listen thread (see below) so clearing
        # self.bus cannot cause AttributeError if join times out.
        self._stop_listen.set()
        bus = self.bus
        if bus is not None:
            try:
                bus.shutdown()
            except Exception:
                pass
        if self._listen_thread and self._listen_thread.is_alive():
            self._listen_thread.join(timeout=2.0)
            if self._listen_thread.is_alive() and self.logger:
                self.logger.warning(
                    "CAN listen thread did not exit after shutdown; continuing close"
                )
        self._listen_thread = None
        self.bus = None

    def test_connection(self) -> bool:
        """Consider connected if status was recently seen, or after a brief wait."""
        if self.is_alive():
            return True
        self.ping()
        deadline = time.time() + 0.5
        while time.time() < deadline:
            if self.is_alive():
                return True
            time.sleep(0.05)
        return self.is_alive()

    def is_alive(self) -> bool:
        with self._status_lock:
            if self._status.last_update <= 0:
                return False
            return (time.time() - self._status.last_update) < 0.5

    def set_duty(self, duty: float) -> bool:
        return self._send_command(CAN_PACKET_SET_DUTY, int(duty * 100000))

    def set_rpm(self, rpm: float) -> bool:
        return self._send_command(CAN_PACKET_SET_RPM, int(rpm))

    def set_current(self, current: float) -> bool:
        return self._send_command(CAN_PACKET_SET_CURRENT, int(current * 1000))

    def set_current_brake(self, current: float) -> bool:
        return self._send_command(CAN_PACKET_SET_CURRENT_BRAKE, int(current * 1000))

    def ping(self) -> bool:
        return self._send_command(CAN_PACKET_PING, 0)

    def get_status(self) -> Optional[VescStatus]:
        with self._status_lock:
            if self._status.last_update <= 0:
                return None
            # Return a shallow copy so callers do not mutate shared state.
            return VescStatus(
                rpm=self._status.rpm,
                current=self._status.current,
                duty_cycle=self._status.duty_cycle,
                amp_hours=self._status.amp_hours,
                amp_hours_charged=self._status.amp_hours_charged,
                watt_hours=self._status.watt_hours,
                watt_hours_charged=self._status.watt_hours_charged,
                fet_temp=self._status.fet_temp,
                motor_temp=self._status.motor_temp,
                current_in=self._status.current_in,
                pid_pos=self._status.pid_pos,
                tachometer=self._status.tachometer,
                input_voltage=self._status.input_voltage,
                last_update=self._status.last_update,
            )

    def _build_extended_id(self, command: int) -> int:
        return ((command & 0xFF) << 8) | (self.vesc_id & 0xFF)

    def _send_command(self, command: int, value: int) -> bool:
        if self.bus is None:
            if self.logger:
                self.logger.error("CAN bus not open")
            return False
        try:
            data = struct.pack(">i", value)
            msg = can.Message(
                arbitration_id=self._build_extended_id(command),
                data=data,
                is_extended_id=True,
            )
            if self._debug and self.logger:
                self.logger.debug(
                    f"Sending CAN cmd={command} id={self.vesc_id} data={data.hex()}"
                )
            self.bus.send(msg)
            return True
        except Exception as e:
            if self.logger:
                self.logger.error(f"Failed to send CAN command: {e}")
            return False

    def _listen_for_messages(self) -> None:
        # Local reference so close() can clear self.bus without racing this loop.
        bus = self.bus
        if bus is None:
            return
        while not self._stop_listen.is_set():
            try:
                msg = bus.recv(timeout=0.1)
            except Exception as e:
                if self._stop_listen.is_set():
                    return
                if self.logger:
                    self.logger.warning(f"Error receiving CAN frame: {e}")
                continue
            if msg is None:
                continue
            if not msg.is_extended_id:
                continue
            self._handle_status_message(msg)

    def _handle_status_message(self, msg: can.Message) -> None:
        command = (msg.arbitration_id >> 8) & 0xFF
        sender_id = msg.arbitration_id & 0xFF
        if sender_id != self.vesc_id:
            return

        data = bytes(msg.data)
        if command == CAN_PACKET_PONG:
            if self.logger:
                self.logger.info(f"Received PONG from VESC ID {sender_id}")
            with self._status_lock:
                self._status.last_update = time.time()
            return

        if len(data) < 8:
            return

        with self._status_lock:
            if command == CAN_PACKET_STATUS:
                self._status.rpm = float(struct.unpack(">i", data[0:4])[0])
                self._status.current = struct.unpack(">h", data[4:6])[0] / 10.0
                self._status.duty_cycle = struct.unpack(">h", data[6:8])[0] / 1000.0
            elif command == CAN_PACKET_STATUS_2:
                self._status.amp_hours = struct.unpack(">i", data[0:4])[0] / 10000.0
                self._status.amp_hours_charged = (
                    struct.unpack(">i", data[4:8])[0] / 10000.0
                )
            elif command == CAN_PACKET_STATUS_3:
                self._status.watt_hours = struct.unpack(">i", data[0:4])[0] / 10000.0
                self._status.watt_hours_charged = (
                    struct.unpack(">i", data[4:8])[0] / 10000.0
                )
            elif command == CAN_PACKET_STATUS_4:
                self._status.fet_temp = struct.unpack(">h", data[0:2])[0] / 10.0
                self._status.motor_temp = struct.unpack(">h", data[2:4])[0] / 10.0
                self._status.current_in = struct.unpack(">h", data[4:6])[0] / 10.0
                self._status.pid_pos = struct.unpack(">h", data[6:8])[0] / 50.0
            elif command == CAN_PACKET_STATUS_5:
                self._status.tachometer = float(struct.unpack(">i", data[0:4])[0])
                self._status.input_voltage = struct.unpack(">h", data[4:6])[0] / 10.0
            else:
                return

            self._status.last_update = time.time()
