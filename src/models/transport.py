"""Transport abstraction for VESC serial and SocketCAN I/O."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, Mapping, Optional


@dataclass
class VescStatus:
    """Cached telemetry shared across transports where available."""

    rpm: float = 0.0
    current: float = 0.0
    duty_cycle: float = 0.0
    amp_hours: float = 0.0
    amp_hours_charged: float = 0.0
    watt_hours: float = 0.0
    watt_hours_charged: float = 0.0
    fet_temp: float = 0.0
    motor_temp: float = 0.0
    current_in: float = 0.0
    pid_pos: float = 0.0
    tachometer: float = 0.0
    # STATUS_5 as four big-endian uint16 words (A,B,C,D). B is the low half
    # of the int32 tachometer and is what typically changes first.
    status5_a: float = 0.0
    status5_b: float = 0.0
    status5_c: float = 0.0
    status5_d: float = 0.0
    input_voltage: float = 0.0
    last_update: float = 0.0
    status5_last_update: float = 0.0
    raw_hex: Optional[str] = None
    extra: Dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {
            "rpm": self.rpm,
            "current": self.current,
            "duty_cycle": self.duty_cycle,
            "amp_hours": self.amp_hours,
            "amp_hours_charged": self.amp_hours_charged,
            "watt_hours": self.watt_hours,
            "watt_hours_charged": self.watt_hours_charged,
            "fet_temp": self.fet_temp,
            "motor_temp": self.motor_temp,
            "current_in": self.current_in,
            "pid_pos": self.pid_pos,
            "tachometer": self.tachometer,
            "status5_a": self.status5_a,
            "status5_b": self.status5_b,
            "status5_c": self.status5_c,
            "status5_d": self.status5_d,
            "input_voltage": self.input_voltage,
            "last_update": self.last_update,
            "status5_last_update": self.status5_last_update,
        }


class VescTransport(ABC):
    """Logical VESC operations; each backend encodes its own wire format."""

    @abstractmethod
    def open(self) -> None:
        ...

    @abstractmethod
    def close(self) -> None:
        ...

    @abstractmethod
    def test_connection(self) -> bool:
        ...

    @abstractmethod
    def set_duty(self, duty: float) -> bool:
        ...

    @abstractmethod
    def set_rpm(self, rpm: float) -> bool:
        ...

    @abstractmethod
    def set_current(self, current: float) -> bool:
        ...

    @abstractmethod
    def set_current_brake(self, current: float) -> bool:
        ...

    @abstractmethod
    def ping(self) -> bool:
        ...

    @abstractmethod
    def get_status(self) -> Optional[VescStatus]:
        ...

    def get_tachometer(self) -> Optional[float]:
        """CAN STATUS_5 tachometer when available; otherwise None."""
        return None

    def clear_buffers(self) -> bool:
        """Clear I/O buffers when supported (serial only by default)."""
        return False

    def set_debug(self, debug: bool) -> None:
        self._debug = bool(debug)


def create_transport(attributes: Mapping[str, Any], logger: Any) -> VescTransport:
    """Build a transport from component attributes."""
    transport_name = str(attributes.get("transport", "serial")).lower()
    debug = bool(attributes.get("debug", True))

    if transport_name == "serial":
        from .serial_transport import SerialTransport

        return SerialTransport(
            port=str(attributes.get("port", "/dev/ttyACM0")),
            baudrate=int(attributes.get("baudrate", 115200)),
            timeout=float(attributes.get("timeout", 1.0)),
            duty_cycle_format=str(attributes.get("duty_cycle_format", "int")),
            debug=debug,
            logger=logger,
        )

    if transport_name == "can":
        from .can_transport import CanTransport

        interface = attributes.get("interface")
        vesc_id = attributes.get("id")
        if not interface:
            raise ValueError("CAN transport requires 'interface' (e.g. 'can0')")
        if vesc_id is None:
            raise ValueError("CAN transport requires 'id' (VESC controller ID 0-255)")
        vesc_id_int = int(vesc_id)
        if vesc_id_int < 0 or vesc_id_int > 255:
            raise ValueError("CAN 'id' must be between 0 and 255")

        return CanTransport(
            interface=str(interface),
            vesc_id=vesc_id_int,
            debug=debug,
            logger=logger,
        )

    raise ValueError(
        f"Unsupported transport '{transport_name}'; use 'serial' or 'can'"
    )


def validate_transport_attributes(attributes: Mapping[str, Any]) -> None:
    """Raise ValueError if transport-related attributes are invalid."""
    transport_name = str(attributes.get("transport", "serial")).lower()
    if transport_name not in ("serial", "can"):
        raise ValueError(
            f"Unsupported transport '{transport_name}'; use 'serial' or 'can'"
        )
    if transport_name == "can":
        if not attributes.get("interface"):
            raise ValueError("CAN transport requires 'interface' (e.g. 'can0')")
        if attributes.get("id") is None:
            raise ValueError("CAN transport requires 'id' (VESC controller ID 0-255)")
        vesc_id = int(attributes["id"])
        if vesc_id < 0 or vesc_id > 255:
            raise ValueError("CAN 'id' must be between 0 and 255")
