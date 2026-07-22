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
        self._rpm_task = None
        self._target_rpm = 0.0
        self._stop_rpm_task = False
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
        self._warned_missing_status5 = False
        # Closed-loop mechanical RPM: map shaft RPM → duty (same actuator as SetPower).
        # Tach is for GetPosition / GoFor distance only — not for torque feedback.
        # Current-mode + sparse STATUS_5 was erratic; duty keepalive matches SetPower.
        self._closed_loop_rpm = True
        self._rpm_kp = 0.0  # optional duty trim per RPM error (leave 0)
        self._rpm_ki = 0.0
        self._rpm_max_duty = 1.0
        self._rpm_duty_ref = 150.0  # |duty| ≈ |rpm| / rpm_duty_ref
        self._rpm_integral = 0.0
        self._rpm_cmd_duty = 0.0
        self._measured_rpm = 0.0
        self._rpm_tach_last: Optional[float] = None
        self._rpm_tach_last_ts: Optional[float] = None
        self._rpm_meas_last_ts: Optional[float] = None
        # Proportional RPM setpoint ramp (preserves L/R ratio for SetVelocity).
        self._rpm_goal = 0.0
        self._rpm_setpoint = 0.0
        self._rpm_ramp_from = 0.0
        self._rpm_ramp_progress = 1.0

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
        # Sync path: cancel keepalive tasks, then swap transports.
        self._stop_power_task = True
        self._stop_rpm_task = True
        if self._power_task and not self._power_task.done():
            self._power_task.cancel()
        self._power_task = None
        if self._rpm_task and not self._rpm_task.done():
            self._rpm_task.cancel()
        self._rpm_task = None

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
        self._warned_missing_status5 = False
        # Default on for CAN so SetRPM is mechanical RPM via tach feedback.
        self._closed_loop_rpm = _as_bool(
            attributes.get("closed_loop_rpm"),
            self._transport_name == "can",
        )
        self._rpm_kp = float(attributes.get("rpm_kp", 0.0))
        self._rpm_ki = float(attributes.get("rpm_ki", 0.0))
        self._rpm_max_duty = float(attributes.get("rpm_max_duty", 1.0))
        if self._rpm_max_duty <= 0 or self._rpm_max_duty > 1.0:
            self._rpm_max_duty = 1.0
        self._rpm_duty_ref = float(attributes.get("rpm_duty_ref", 150.0))
        if self._rpm_duty_ref < 10.0:
            self._rpm_duty_ref = 10.0
        self._rpm_integral = 0.0
        self._rpm_cmd_duty = 0.0
        self._measured_rpm = 0.0
        self._rpm_tach_last = None
        self._rpm_tach_last_ts = None
        self._rpm_meas_last_ts = None
        self._rpm_goal = 0.0
        self._rpm_setpoint = 0.0
        self._rpm_ramp_from = 0.0
        self._rpm_ramp_progress = 1.0
        self.logger.info(
            "VESC config: transport=%s ramp_up_enabled=%s ramp_up_rate=%s "
            "command_interval=%s ticks_per_rotation=%s closed_loop_rpm=%s "
            "rpm_kp=%s rpm_ki=%s rpm_max_duty=%s rpm_duty_ref=%s",
            self._transport_name,
            self._ramp_up_enabled,
            self._ramp_up_rate,
            self._command_interval,
            self._ticks_per_rotation,
            self._closed_loop_rpm,
            self._rpm_kp,
            self._rpm_ki,
            self._rpm_max_duty,
            self._rpm_duty_ref,
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
        """Stop the duty keepalive/ramp task without holding `_lock` during await."""
        async with self._lock:
            self._stop_power_task = True
            task = self._power_task
            self._power_task = None
        await self._await_halted_task(task)

    async def _halt_rpm_task(self) -> None:
        """Stop the RPM keepalive task without holding `_lock` during await."""
        async with self._lock:
            self._stop_rpm_task = True
            task = self._rpm_task
            self._rpm_task = None
        await self._await_halted_task(task)

    async def _halt_control_tasks(self) -> None:
        """Stop duty and RPM keepalives (VESC CAN times out ~0.5s without repeats)."""
        await self._halt_power_task()
        await self._halt_rpm_task()

    async def _await_halted_task(self, task) -> None:
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

    def _can_use_rpm_closed_loop(self) -> bool:
        """True when we should treat SetRPM as mechanical RPM with tach feedback."""
        if not self._closed_loop_rpm:
            return False
        if self._transport_name != "can" or self._transport is None:
            return False
        return self._transport.get_tachometer() is not None

    def _start_rpm_keepalive(self, rpm: float, *, allow_ramp: bool = True) -> None:
        """Caller must hold `_lock`. Starts RPM keepalive / closed-loop speed task.

        ``allow_ramp``: SetVelocity keeps proportional setpoint ramping so L/R
        ratio holds. GoFor/MoveStraight/Spin must NOT ramp from the previous
        move — ramping left from a prior spin (-rpm) toward +rpm while right
        stays +rpm makes the base yaw/weave ("multiple directions").
        """
        self._rpm_goal = rpm
        self._target_rpm = rpm
        self._is_powered = rpm != 0.0
        self._is_moving = rpm != 0.0
        self._rpm_integral = 0.0
        # Drop opposite-direction duty immediately (no slew through reverse).
        if rpm == 0.0 or self._rpm_cmd_duty * rpm < 0:
            self._rpm_cmd_duty = 0.0
            self._current_power = 0.0

        if rpm == 0.0:
            self._rpm_setpoint = 0.0
            self._rpm_ramp_from = 0.0
            self._rpm_ramp_progress = 1.0
            if self._transport is not None:
                self._transport.set_duty(0.0)
            return

        if (
            allow_ramp
            and self._ramp_up_enabled
            and self._ramp_up_rate > 0
            and rpm != self._rpm_setpoint
        ):
            if self._rpm_setpoint * rpm < 0:
                self._rpm_ramp_from = 0.0
            else:
                self._rpm_ramp_from = self._rpm_setpoint
            self._rpm_ramp_progress = 0.0
        else:
            self._rpm_ramp_from = rpm
            self._rpm_ramp_progress = 1.0
            self._rpm_setpoint = rpm

        self._stop_rpm_task = False
        self._rpm_task = asyncio.create_task(self._rpm_control_task())

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
        await self._halt_control_tasks()

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
        """Go for a specific number of revolutions at a given RPM.

        Direction follows the Viam motor API: negative ``rpm`` *or* negative
        ``revolutions`` means backward; if *both* are negative the motor goes
        forward. Wheeled-base ``Spin`` relies on this — it calls
        ``GoFor(-rpm, +revs)`` on the left and ``GoFor(+rpm, +revs)`` on the
        right.

        On CAN with tachometer odometry, runs closed-loop until the measured
        position reaches the target. Serial (no encoder) falls back to timed motion.
        """
        if revolutions == 0 or rpm == 0:
            await self.stop()
            return

        # Same-sign rpm/revolutions → forward; opposite signs → backward.
        # (Do not ignore rpm sign: that made Spin drive both wheels forward.)
        travel = abs(revolutions) if (rpm > 0) == (revolutions > 0) else -abs(revolutions)
        signed_rpm = abs(rpm) if travel > 0 else -abs(rpm)
        start = await self.get_position()
        target = start + travel

        await self._halt_control_tasks()
        async with self._lock:
            transport = self._require_transport()
            closed = self._can_use_rpm_closed_loop()
            if closed:
                self._start_rpm_keepalive(signed_rpm, allow_ramp=False)
            else:
                if not transport.set_rpm(signed_rpm):
                    raise RuntimeError("Failed to set RPM for GoFor")
                self._start_rpm_keepalive(signed_rpm, allow_ramp=False)

        self.logger.info(
            "GoFor: start=%.3f target=%.3f rev=%.3f travel=%.3f rpm=%.1f",
            start,
            target,
            revolutions,
            travel,
            signed_rpm,
        )

        # Prefer closed-loop on CAN tachometer; timed fallback otherwise.
        use_odometry = (
            self._transport_name == "can"
            and self._transport is not None
            and self._transport.get_tachometer() is not None
        )

        try:
            if use_odometry:
                await self._wait_for_position(
                    start=start,
                    target=target,
                    moving_positive=travel > 0,
                    distance_revs=abs(travel),
                    rpm_abs=abs(rpm),
                    timeout=timeout,
                )
            else:
                time_needed = abs(travel / rpm) * 60.0
                if timeout is not None and timeout > 0:
                    time_needed = min(time_needed, timeout)
                self.logger.warning(
                    "GoFor using timed motion (%.2fs); no tachometer odometry",
                    time_needed,
                )
                await asyncio.sleep(time_needed)
                async with self._lock:
                    self._position = target
        finally:
            # Always drop keepalive / duty so a failed GoFor cannot leave the motor spinning.
            await self.stop()

    async def _wait_for_position(
        self,
        *,
        start: float,
        target: float,
        moving_positive: bool,
        distance_revs: float,
        rpm_abs: float,
        timeout: Optional[float] = None,
    ) -> None:
        """Poll GetPosition until target is reached or timeout.

        Raises TimeoutError if the target is not reached in time.
        """
        # Allow 2x nominal time plus 2s margin; poll at ~50Hz.
        nominal_s = (distance_revs / max(rpm_abs, 1e-6)) * 60.0
        wait_s = max(nominal_s * 2.0, 2.0) + 2.0
        if timeout is not None and timeout > 0:
            wait_s = min(wait_s, timeout)
        deadline = time.monotonic() + wait_s

        # One tach tick in revolutions. Cap tolerance below half the move so the
        # start position cannot satisfy "reached" (e.g. 0.01 rev with tol 0.02).
        tick_rev = 1.0 / max(abs(self._ticks_per_rotation), 1e-9)
        tolerance = max(tick_rev, min(0.02, distance_revs * 0.25))
        if distance_revs > 0:
            tolerance = min(tolerance, max(distance_revs * 0.5, tick_rev * 0.5))

        last_logged = 0.0
        while time.monotonic() < deadline:
            pos = await self.get_position()
            if moving_positive and pos >= target - tolerance:
                self.logger.info(
                    "GoFor/GoTo reached target pos=%.3f (target=%.3f tol=%.4f)",
                    pos,
                    target,
                    tolerance,
                )
                return
            if not moving_positive and pos <= target + tolerance:
                self.logger.info(
                    "GoFor/GoTo reached target pos=%.3f (target=%.3f tol=%.4f)",
                    pos,
                    target,
                    tolerance,
                )
                return

            # Detect inverted tach vs SET_RPM sign early.
            if moving_positive and pos < start - max(tolerance, 0.05):
                self.logger.warning(
                    "GoFor/GoTo position moving opposite to commanded +RPM "
                    "(start=%.3f pos=%.3f). Check motor direction / ticks sign.",
                    start,
                    pos,
                )
            elif not moving_positive and pos > start + max(tolerance, 0.05):
                self.logger.warning(
                    "GoFor/GoTo position moving opposite to commanded -RPM "
                    "(start=%.3f pos=%.3f). Check motor direction / ticks sign.",
                    start,
                    pos,
                )

            now = time.monotonic()
            if now - last_logged >= 0.5:
                self.logger.info(
                    "GoFor/GoTo progress pos=%.3f target=%.3f remaining=%.3f",
                    pos,
                    target,
                    (target - pos) if moving_positive else (pos - target),
                )
                last_logged = now
            await asyncio.sleep(0.02)

        pos = await self.get_position()
        raise TimeoutError(
            f"GoFor/GoTo timed out at pos={pos:.3f} (target={target:.3f}, "
            f"start={start:.3f}, waited={wait_s:.1f}s)"
        )

    async def go_to(
        self,
        rpm: float,
        position_revolutions: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Go to an absolute position (revolutions) at a given RPM."""
        current_position = await self.get_position()
        revolutions_needed = position_revolutions - current_position
        # Direction comes from the position delta; |rpm| is speed only so a
        # negative rpm argument cannot invert GoFor's XOR sign rule.
        self.logger.info(
            "GoTo: current=%.3f target=%.3f delta=%.3f rpm=%.1f",
            current_position,
            position_revolutions,
            revolutions_needed,
            rpm,
        )
        await self.go_for(
            abs(rpm),
            revolutions_needed,
            extra=extra,
            timeout=timeout,
            **kwargs,
        )

    async def set_rpm(
        self,
        rpm: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Set motor speed.

        With CAN + tachometer and closed_loop_rpm (default on CAN), ``rpm`` is
        mechanical shaft RPM: duty is commanded from the setpoint (same path as
        SetPower). Tachometer is used for position / GoFor distance, not torque.
        Otherwise ``rpm`` is sent as open-loop VESC ERPM (with keepalive).
        """
        await self._halt_control_tasks()
        async with self._lock:
            try:
                transport = self._require_transport()
                closed = self._can_use_rpm_closed_loop()
                if closed:
                    self._start_rpm_keepalive(rpm)
                    self.logger.info(
                        "Set mechanical RPM to %.1f (closed-loop duty map)", rpm
                    )
                else:
                    if not transport.set_rpm(rpm):
                        raise RuntimeError("Failed to set RPM")
                    self._start_rpm_keepalive(rpm)
                    if self._closed_loop_rpm and self._transport_name == "can":
                        self.logger.warning(
                            "SetRPM %.1f as open-loop ERPM (no STATUS_5 tach yet); "
                            "closed-loop mechanical RPM needs tachometer",
                            rpm,
                        )
                    else:
                        self.logger.info("Set open-loop ERPM to %.1f", rpm)
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

        On CAN, position uses STATUS_5 tachometer (int32):
        (tachometer - zero) / ticks_per_rotation + offset.
        Serial keeps the software position counter.
        """
        if self._transport_name == "can" and self._transport is not None:
            tach = self._transport.get_tachometer()
            if tach is None:
                if not self._warned_missing_status5:
                    self._warned_missing_status5 = True
                    self.logger.warning(
                        "GetPosition is 0: module has not received STATUS_5 yet "
                        "for this VESC id. Confirm attributes.id matches the id "
                        "in your can dump (0x1B00|id). DoCommand "
                        '{"command":"get_position_debug"}.'
                    )
                return self._position
            async with self._lock:
                if not self._tach_zero_initialized:
                    # First reading defines the zero unless ResetZeroPosition ran.
                    self._tach_zero = tach
                    self._tach_zero_initialized = True
                    self.logger.info(
                        "CAN position zero locked at tachometer=%.0f "
                        "(ticks_per_rotation=%s)",
                        tach,
                        self._ticks_per_rotation,
                    )
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
        await self._halt_control_tasks()

        async with self._lock:
            self._require_transport().set_duty(0.0)

            self._target_power = 0.0
            self._target_rpm = 0.0
            self._rpm_goal = 0.0
            self._rpm_setpoint = 0.0
            self._rpm_ramp_progress = 1.0
            self._is_powered = False
            self._current_power = 0.0
            self._current_rpm = 0.0
            self._measured_rpm = 0.0
            self._rpm_integral = 0.0
            self._rpm_cmd_duty = 0.0
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
                "measured_rpm": self._measured_rpm,
                "target_rpm": self._target_rpm,
                "rpm_setpoint": self._rpm_setpoint,
                "rpm_ramp_progress": self._rpm_ramp_progress,
                "rpm_cmd_duty": self._rpm_cmd_duty,
                "rpm_duty_ref": self._rpm_duty_ref,
                "closed_loop_rpm": self._closed_loop_rpm,
                "is_moving": self._is_moving,
                "position": self._position,
                "debug_mode": self._debug_mode,
                "transport": self._transport_name,
                "ticks_per_rotation": self._ticks_per_rotation,
            }
            if transport:
                vesc_status = transport.get_status()
                if vesc_status is not None and vesc_status.raw_hex is None:
                    result["vesc"] = vesc_status.as_dict()
                get_dbg = getattr(transport, "get_status5_debug", None)
                if callable(get_dbg):
                    result["status5"] = get_dbg()
            return result

        elif command_name == "get_position_debug":
            dbg: Dict[str, Any] = {
                "status": "success",
                "transport": self._transport_name,
                "ticks_per_rotation": self._ticks_per_rotation,
                "tach_zero": self._tach_zero,
                "tach_zero_initialized": self._tach_zero_initialized,
                "position_offset": self._position_offset,
                "position": self._position,
            }
            if transport and hasattr(transport, "get_status5_debug"):
                dbg["status5"] = transport.get_status5_debug()
                tach = transport.get_tachometer()
                dbg["tachometer"] = tach
                if tach is not None and self._tach_zero_initialized:
                    dbg["position_calc"] = (
                        (tach - self._tach_zero) / self._ticks_per_rotation
                        + self._position_offset
                    )
            return dbg

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
            await self._halt_control_tasks()
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
                self._target_rpm = 0.0
                self._rpm_goal = 0.0
                self._rpm_setpoint = 0.0
                self._rpm_ramp_progress = 1.0
                self._is_powered = False
                self._current_power = 0.0
                self._current_rpm = 0.0
                self._measured_rpm = 0.0
                self._rpm_integral = 0.0
                self._rpm_cmd_duty = 0.0
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
        await self._halt_control_tasks()
        async with self._lock:
            transport = self._transport
            self._transport = None
        if transport:
            transport.close()

    async def _rpm_control_task(self):
        """Keepalive + closed-loop mechanical RPM via setpoint → duty.

        Closed-loop (CAN + tach present): map shaft RPM setpoint to duty cycle
        (same actuator SetPower uses). Tach updates measured RPM for telemetry;
        GoFor still stops on tach position. Open-loop: re-send SET_RPM (ERPM).

        GoFor disables setpoint ramp. SetVelocity may ramp setpoints so L/R
        ratio stays constant.
        """
        last_logged = 0.0
        while True:
            try:
                async with self._lock:
                    if self._stop_rpm_task:
                        return
                    transport = self._transport
                    goal = self._rpm_goal
                    interval = self._command_interval
                    ticks = self._ticks_per_rotation
                    kp = self._rpm_kp
                    ki = self._rpm_ki
                    max_duty = self._rpm_max_duty
                    duty_ref = self._rpm_duty_ref
                    ramp_enabled = self._ramp_up_enabled
                    ramp_rate = self._ramp_up_rate
                    ramp_from = self._rpm_ramp_from
                    ramp_progress = self._rpm_ramp_progress
                    closed = (
                        self._closed_loop_rpm
                        and self._transport_name == "can"
                        and transport is not None
                    )
                    integral = self._rpm_integral
                    tach_last = self._rpm_tach_last
                    tach_last_ts = self._rpm_tach_last_ts
                    measured = self._measured_rpm

                if transport is None:
                    await asyncio.sleep(interval)
                    continue

                if ramp_enabled and ramp_rate > 0 and ramp_progress < 1.0:
                    ramp_progress = min(1.0, ramp_progress + ramp_rate * interval)
                    setpoint = ramp_from + (goal - ramp_from) * ramp_progress
                else:
                    ramp_progress = 1.0
                    setpoint = goal

                if not closed or transport.get_tachometer() is None:
                    transport.set_rpm(setpoint)
                    async with self._lock:
                        self._rpm_setpoint = setpoint
                        self._rpm_ramp_progress = ramp_progress
                        self._target_rpm = goal
                        self._current_rpm = setpoint
                    await asyncio.sleep(interval)
                    continue

                tach = transport.get_tachometer()
                now = time.monotonic()

                # Telemetry-only speed estimate (does not drive torque).
                if tach is not None and tach_last is None:
                    tach_last = tach
                    tach_last_ts = now
                    measured = 0.0
                elif (
                    tach is not None
                    and tach_last is not None
                    and tach != tach_last
                    and tach_last_ts is not None
                ):
                    real_dt = max(now - tach_last_ts, 1e-3)
                    real_dt = min(real_dt, 0.5)
                    drevs = (tach - tach_last) / ticks
                    instant = (drevs / real_dt) * 60.0
                    measured = 0.8 * measured + 0.2 * instant
                    tach_last = tach
                    tach_last_ts = now
                elif tach_last_ts is not None and (now - tach_last_ts) > 0.6:
                    measured *= 0.9
                    if abs(measured) < 0.5:
                        measured = 0.0

                # Duty feedforward from setpoint — same class of command as SetPower.
                if abs(setpoint) < 1e-9 or duty_ref < 1e-9:
                    duty = 0.0
                else:
                    duty = setpoint / duty_ref
                    # Small crawl floor so tiny RPM still moves (scaled in).
                    floor = 0.05 * min(1.0, abs(setpoint) / 25.0)
                    if 0.0 < abs(duty) < floor:
                        duty = floor if duty > 0 else -floor
                duty = max(-max_duty, min(max_duty, duty))

                # Optional slow tach trim (default gains are 0).
                if kp != 0.0 or ki != 0.0:
                    error = setpoint - measured
                    integral += error * interval
                    if ki > 1e-9:
                        i_lim = (0.2 * max_duty) / ki
                        integral = max(-i_lim, min(i_lim, integral))
                    trim = kp * error + ki * integral
                    trim = max(-0.2 * max_duty, min(0.2 * max_duty, trim))
                    duty = max(-max_duty, min(max_duty, duty + trim))
                    # Never reverse against setpoint.
                    if setpoint > 0.0 and duty < 0.0:
                        duty = 0.0
                    elif setpoint < 0.0 and duty > 0.0:
                        duty = 0.0

                transport.set_duty(duty)

                async with self._lock:
                    self._rpm_integral = integral
                    self._rpm_cmd_duty = duty
                    self._rpm_setpoint = setpoint
                    self._rpm_ramp_progress = ramp_progress
                    self._target_rpm = goal
                    self._measured_rpm = measured
                    self._current_rpm = measured
                    self._current_power = duty
                    self._rpm_tach_last = tach_last
                    self._rpm_tach_last_ts = tach_last_ts
                    self._is_moving = abs(measured) > 0.5 or abs(duty) > 0.02

                if now - last_logged >= 0.5:
                    self.logger.info(
                        "RPM loop: goal=%.1f set=%.1f meas=%.1f duty=%.3f prog=%.2f",
                        goal,
                        setpoint,
                        measured,
                        duty,
                        ramp_progress,
                    )
                    last_logged = now

                await asyncio.sleep(interval)
            except asyncio.CancelledError:
                raise
            except Exception as e:
                self.logger.error(f"RPM control error: {e}")
                await asyncio.sleep(self._command_interval)

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