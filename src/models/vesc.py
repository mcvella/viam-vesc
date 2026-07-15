import asyncio
import time
from dataclasses import dataclass
from typing import (Any, ClassVar, Dict, List, Mapping, Optional, Sequence,
                    Tuple)

from typing_extensions import Self
from viam.components.motor import *
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict

from .transport import VescTransport, create_transport, validate_transport_attributes


def _as_bool(value: Any, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


class Vesc(Motor, EasyResource):
    # To enable debug-level logging, either run viam-server with the --debug option,
    # or configure your resource/machine to display debug logs.
    MODEL: ClassVar[Model] = Model(ModelFamily("mcvella", "vesc"), "vesc")

    def __init__(self, name: str):
        super().__init__(name)
        self._transport: Optional[VescTransport] = None
        self._is_powered = False
        self._current_power = 0.0
        self._current_rpm = 0.0
        self._position = 0.0
        self._is_moving = False
        self._lock = asyncio.Lock()
        self._debug_mode = True
        self._power_task = None
        self._target_power = 0.0
        self._stop_power_task = False
        self._command_interval = 0.01  # 10ms default, configurable
        self._ramp_up_enabled = True
        self._ramp_up_rate = 0.25  # Power change per second
        self._current_ramp_power = 0.0
        self._ramp_last_ts = 0.0
        self._direction_change_in_progress = False
        self._duty_cycle_format = "int"
        self._transport_name = "serial"
        # CAN odometry: revolutions = (tachometer - tach_zero) / ticks_per_rotation
        self._ticks_per_rotation = 1.0
        self._tach_zero = 0.0
        self._tach_zero_initialized = False
        self._position_offset = 0.0

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """This method creates a new instance of this Motor component.
        The default implementation sets the name from the `config` parameter and then calls `reconfigure`.

        Args:
            config (ComponentConfig): The configuration for this resource
            dependencies (Mapping[ResourceName, ResourceBase]): The dependencies (both required and optional)

        Returns:
            Self: The resource
        """
        vesc = cls(config.name)
        vesc.reconfigure(config, dependencies)
        return vesc

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        """This method allows you to validate the configuration object received from the machine,
        as well as to return any required dependencies or optional dependencies based on that `config`.

        Args:
            config (ComponentConfig): The configuration for this resource

        Returns:
            Tuple[Sequence[str], Sequence[str]]: A tuple where the
                first element is a list of required dependencies and the
                second element is a list of optional dependencies
        """
        attributes = struct_to_dict(config.attributes)
        validate_transport_attributes(attributes)
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both required and optional)
        """
        # Sync path: cancel the keepalive task, then swap transports.
        self._stop_power_task = True
        if self._power_task and not self._power_task.done():
            self._power_task.cancel()
        self._power_task = None

        old_transport = self._transport
        self._transport = None
        if old_transport:
            old_transport.close()

        attributes = struct_to_dict(config.attributes)

        self._debug_mode = _as_bool(attributes.get("debug"), True)
        self._command_interval = float(attributes.get("command_interval", 0.01))
        if self._command_interval <= 0:
            self._command_interval = 0.01
        self._ramp_up_enabled = _as_bool(attributes.get("ramp_up_enabled"), True)
        self._ramp_up_rate = float(attributes.get("ramp_up_rate", 0.25))
        if self._ramp_up_rate <= 0:
            self._ramp_up_rate = 0.25
        self._duty_cycle_format = attributes.get("duty_cycle_format", "int")
        self._transport_name = str(attributes.get("transport", "serial")).lower()
        self._ticks_per_rotation = float(attributes.get("ticks_per_rotation", 1.0))
        if self._ticks_per_rotation == 0:
            raise ValueError("ticks_per_rotation must be non-zero")
        self._tach_zero = 0.0
        self._tach_zero_initialized = False
        self._position_offset = 0.0
        self.logger.info(
            "VESC config: transport=%s ramp_up_enabled=%s ramp_up_rate=%s "
            "command_interval=%s ticks_per_rotation=%s",
            self._transport_name,
            self._ramp_up_enabled,
            self._ramp_up_rate,
            self._command_interval,
            self._ticks_per_rotation,
        )

        try:
            self._transport = create_transport(attributes, self.logger)
            self._transport.open()

            if self._transport.test_connection():
                self.logger.info("VESC connection test successful")
            else:
                self.logger.warning(
                    "VESC connection test failed - motor may not respond"
                )
        except Exception as e:
            self.logger.error(f"Failed to connect to VESC: {e}")
            if self._transport:
                self._transport.close()
                self._transport = None
            raise

    def _require_transport(self) -> VescTransport:
        if not self._transport:
            raise RuntimeError("VESC transport not open")
        return self._transport

    async def _halt_power_task(self) -> None:
        """Stop the keepalive/ramp task without holding `_lock` during the await."""
        async with self._lock:
            self._stop_power_task = True
            task = self._power_task
            self._power_task = None
        if task and not task.done():
            try:
                await asyncio.wait_for(task, timeout=0.5)
            except (asyncio.TimeoutError, asyncio.CancelledError):
                if not task.done():
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass

    @dataclass
    class Properties(Motor.Properties):
        position_reporting: bool = True

    async def set_power(
        self,
        power: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Set motor power as a percentage (-1.0 to 1.0)"""
        power = max(-1.0, min(1.0, power))  # Clamp to [-1, 1]
        await self._halt_power_task()

        async with self._lock:
            transport = self._require_transport()

            self._target_power = power
            self._is_powered = power != 0.0
            self._is_moving = power != 0.0

            if self._ramp_up_enabled:
                # Keep commanded power as the ramp start; do not jump to target.
                # Wall-clock ramp in the background task (not rate*interval once).
                self._ramp_last_ts = time.monotonic()
                self._current_ramp_power = self._current_power
                transport.set_duty(self._current_power)
                self._stop_power_task = False
                self._power_task = asyncio.create_task(self._simple_power_task())
                est = (
                    abs(power - self._current_power) / self._ramp_up_rate
                    if self._ramp_up_rate > 0
                    else 0.0
                )
                self.logger.info(
                    "Ramping power from %.3f to %.3f (rate=%.3f/s, ~%.2fs)",
                    self._current_power,
                    power,
                    self._ramp_up_rate,
                    est,
                )
            else:
                self._current_power = power
                self._current_ramp_power = power
                transport.set_duty(power)
                if power != 0.0:
                    self._stop_power_task = False
                    self._power_task = asyncio.create_task(self._simple_power_task())
                    self.logger.info(f"Set power to {power:.2f} (ramp disabled)")
                else:
                    self.logger.info("Motor stopped")

    async def go_for(
        self,
        rpm: float,
        revolutions: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Go for a specific number of revolutions at a given RPM"""
        if revolutions == 0:
            await self.stop()
            return

        success = False
        time_needed = 0.0
        async with self._lock:
            transport = self._require_transport()
            try:
                if transport.set_rpm(rpm):
                    success = True
                    self.logger.info(f"Set RPM to {rpm}")
            except Exception as e:
                self.logger.debug(f"RPM control failed: {e}")

            if success:
                self._current_rpm = rpm
                self._is_powered = True
                self._is_moving = True
                if rpm != 0:
                    time_needed = abs(revolutions / rpm) * 60.0

        if success:
            if rpm != 0:
                await asyncio.sleep(time_needed)
                await self.stop()
                async with self._lock:
                    self._position += revolutions
            return

        self.logger.warning("RPM control failed, using power control fallback")
        power = 0.5 if rpm > 0 else -0.5
        await self.set_power(power)
        await asyncio.sleep(2.0)
        await self.stop()

    async def go_to(
        self,
        rpm: float,
        position_revolutions: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Go to a specific position at a given RPM"""
        target_position = position_revolutions
        current_position = self._position
        revolutions_needed = target_position - current_position

        await self.go_for(rpm, revolutions_needed, extra=extra, timeout=timeout, **kwargs)

    async def set_rpm(
        self,
        rpm: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Set the motor RPM"""
        async with self._lock:
            try:
                transport = self._require_transport()
                if transport.set_rpm(rpm):
                    self._current_rpm = rpm
                    self._is_powered = rpm != 0.0
                    self._is_moving = rpm != 0.0
                    self.logger.info(f"Set RPM to {rpm}")
                else:
                    raise RuntimeError("Failed to set RPM")
            except Exception as e:
                self.logger.error(f"Failed to set RPM: {e}")
                raise RuntimeError(f"Failed to set RPM: {e}")

    async def reset_zero_position(
        self,
        offset: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Reset the zero position with an offset"""
        async with self._lock:
            if self._transport_name == "can" and self._transport is not None:
                tach = self._transport.get_tachometer()
                if tach is None:
                    raise RuntimeError(
                        "Cannot reset zero: no CAN STATUS_5 tachometer received yet"
                    )
                self._tach_zero = tach
                self._tach_zero_initialized = True
                self._position_offset = offset
                self._position = offset
                self.logger.info(
                    "Reset CAN zero: tach=%.0f offset=%.3f rev", tach, offset
                )
                return
            self._position = offset
            self.logger.info(f"Reset zero position with offset {offset}")

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> float:
        """Get the current position in revolutions.

        On CAN, position is derived from STATUS_5 tachometer (int32 over words
        A||B; B is the changing low half): 
        (tachometer - tach_zero) / ticks_per_rotation + offset.
        Serial keeps the software position counter.
        """
        if self._transport_name == "can" and self._transport is not None:
            tach = self._transport.get_tachometer()
            if tach is None:
                return self._position
            async with self._lock:
                if not self._tach_zero_initialized:
                    # First reading defines the zero unless ResetZeroPosition ran.
                    self._tach_zero = tach
                    self._tach_zero_initialized = True
                pos = (
                    (tach - self._tach_zero) / self._ticks_per_rotation
                    + self._position_offset
                )
                self._position = pos
                return pos
        return self._position

    async def get_properties(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Properties:
        """Get motor properties"""
        return self.Properties(position_reporting=True)

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Stop the motor"""
        await self._halt_power_task()

        async with self._lock:
            self._require_transport().set_duty(0.0)

            self._target_power = 0.0
            self._is_powered = False
            self._current_power = 0.0
            self._current_rpm = 0.0
            self._is_moving = False
            self.logger.info("Motor stopped")

    async def is_powered(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[bool, float]:
        """Check if the motor is powered and return current power"""
        async with self._lock:
            return self._is_powered, self._current_power

    async def is_moving(self) -> bool:
        """Check if the motor is moving"""
        return self._is_moving

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        """Handle custom commands"""
        command_name = command.get("command")
        transport = self._transport

        if command_name == "get_vesc_values":
            if not transport:
                return {"status": "error", "message": "Transport not open"}
            status = transport.get_status()
            if status is None:
                return {"status": "error", "message": "Failed to read VESC values"}
            result: Dict[str, Any] = {"status": "success"}
            if status.raw_hex is not None:
                result["data"] = status.raw_hex
            else:
                result.update(status.as_dict())
            return result

        elif command_name == "set_current":
            if not transport:
                return {"status": "error", "message": "Transport not open"}
            current = command.get("current", 0.0)
            if isinstance(current, (int, float)):
                if transport.set_current(float(current)):
                    return {"status": "success", "current": current}
                return {"status": "error", "message": "Failed to set current"}
            return {"status": "error", "message": "Invalid current value"}

        elif command_name == "test_connection":
            if not transport:
                return {"status": "error", "message": "Transport not open"}
            if transport.test_connection():
                return {"status": "success", "message": "VESC connection test passed"}
            return {"status": "error", "message": "VESC connection test failed"}

        elif command_name == "ping":
            if not transport:
                return {"status": "error", "message": "Transport not open"}
            if transport.ping():
                return {"status": "success", "message": "Ping sent"}
            return {"status": "error", "message": "Failed to send ping"}

        elif command_name == "set_debug":
            debug = command.get("debug", False)
            self._debug_mode = bool(debug)
            if transport:
                transport.set_debug(self._debug_mode)
            return {"status": "success", "debug_mode": self._debug_mode}

        elif command_name == "get_status":
            result = {
                "status": "success",
                "is_powered": self._is_powered,
                "current_power": self._current_power,
                "current_rpm": self._current_rpm,
                "is_moving": self._is_moving,
                "position": self._position,
                "debug_mode": self._debug_mode,
                "transport": self._transport_name,
            }
            if transport:
                vesc_status = transport.get_status()
                if vesc_status is not None and vesc_status.raw_hex is None:
                    result["vesc"] = vesc_status.as_dict()
            return result

        elif command_name == "set_command_interval":
            interval = command.get("interval", 0.01)
            if isinstance(interval, (int, float)) and 0.001 <= interval <= 0.1:
                self._command_interval = float(interval)
                return {"status": "success", "command_interval": self._command_interval}
            return {
                "status": "error",
                "message": "Invalid interval (must be between 0.001 and 0.1 seconds)",
            }

        elif command_name == "set_ramp_up":
            enabled = command.get("enabled", True)
            self._ramp_up_enabled = _as_bool(enabled, True)
            return {"status": "success", "ramp_up_enabled": self._ramp_up_enabled}

        elif command_name == "set_ramp_rate":
            rate = command.get("rate", 0.25)
            if isinstance(rate, (int, float)) and 0.01 <= rate <= 1.0:
                self._ramp_up_rate = float(rate)
                return {"status": "success", "ramp_up_rate": self._ramp_up_rate}
            return {
                "status": "error",
                "message": "Invalid rate (must be between 0.01 and 1.0 power/second)",
            }

        elif command_name == "get_ramp_status":
            async with self._lock:
                return {
                    "status": "success",
                    "ramp_up_enabled": self._ramp_up_enabled,
                    "ramp_up_rate": self._ramp_up_rate,
                    "current_power": self._current_power,
                    "current_ramp_power": self._current_power,
                    "target_power": self._target_power,
                    "command_interval": self._command_interval,
                }

        elif command_name == "force_stop":
            await self._halt_power_task()
            # Send stop under the lock with no awaits so reconfigure()/close()
            # cannot close the transport between state reset and device commands.
            async with self._lock:
                active_transport = self._transport
                if active_transport is None:
                    return {
                        "status": "error",
                        "message": "Transport not open; cannot force stop VESC",
                    }
                sent_ok = 0
                for _ in range(5):
                    if active_transport.set_duty(0.0):
                        sent_ok += 1
                if sent_ok == 0:
                    return {
                        "status": "error",
                        "message": (
                            "Failed to send stop commands to VESC; "
                            "device may still be running"
                        ),
                    }
                self._target_power = 0.0
                self._is_powered = False
                self._current_power = 0.0
                self._current_rpm = 0.0
                self._is_moving = False
                self._current_ramp_power = 0.0
            return {
                "status": "success",
                "message": "Motor force stopped and state reset",
            }

        elif command_name == "clear_buffers":
            if not transport:
                return {"status": "error", "message": "Transport not open"}
            if transport.clear_buffers():
                return {"status": "success", "message": "Serial buffers cleared"}
            return {
                "status": "error",
                "message": "Buffer clear not supported or transport not open",
            }

        else:
            return {"status": "error", "message": f"Unknown command: {command_name}"}

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> List[Geometry]:
        """Get the geometries of the motor"""
        return []

    async def close(self):
        await self._halt_power_task()
        async with self._lock:
            transport = self._transport
            self._transport = None
        if transport:
            transport.close()

    async def _simple_power_task(self):
        last_logged = 0.0
        while True:
            try:
                async with self._lock:
                    if self._stop_power_task:
                        return
                    if self._ramp_up_enabled:
                        now = time.monotonic()
                        # Cap dt so a stalled event loop cannot leap to the target.
                        dt = min(max(0.0, now - self._ramp_last_ts), 0.05)
                        self._ramp_last_ts = now
                        max_step = self._ramp_up_rate * dt
                        delta = self._target_power - self._current_power
                        if max_step <= 0:
                            # First loop often has dt≈0; wait for the next tick.
                            pass
                        elif abs(delta) > max_step:
                            self._current_power += (
                                max_step if delta > 0 else -max_step
                            )
                        else:
                            self._current_power = self._target_power
                    else:
                        self._current_power = self._target_power

                    self._current_ramp_power = self._current_power

                    # Snapshot under the lock so close/reconfigure cannot swap
                    # transport out from under a mid-flight set_duty call.
                    transport = self._transport
                    duty = self._current_power
                    target = self._target_power
                    interval = self._command_interval
                    ramp_enabled = self._ramp_up_enabled

                if transport is not None:
                    transport.set_duty(duty)

                # Occasional progress logs while a ramp is in flight.
                if ramp_enabled and abs(duty - target) > 1e-4:
                    now = time.monotonic()
                    if now - last_logged >= 0.5:
                        self.logger.info(
                            "Ramp progress: commanded=%.3f target=%.3f", duty, target
                        )
                        last_logged = now

                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self.logger.error(f"Simple power task error: {e}")
                await asyncio.sleep(self._command_interval)
