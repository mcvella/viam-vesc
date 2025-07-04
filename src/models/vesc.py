import asyncio
import struct
import time
from dataclasses import dataclass
from typing import (Any, ClassVar, Dict, Final, List, Mapping, Optional,
                    Sequence, Tuple)

import serial
from typing_extensions import Self
from viam.components.motor import *
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict


class Vesc(Motor, EasyResource):
    # To enable debug-level logging, either run viam-server with the --debug option,
    # or configure your resource/machine to display debug logs.
    MODEL: ClassVar[Model] = Model(ModelFamily("mcvella", "vesc"), "vesc")

    # Standard VESC packet IDs (using more conservative values)
    COMM_FW_VERSION = 0x32
    COMM_JUMP_TO_BOOTLOADER = 0x33
    COMM_ERASE_NEW_APP = 0x34
    COMM_WRITE_NEW_APP_DATA = 0x35
    COMM_GET_VALUES = 0x27
    COMM_SET_DUTY = 5
    COMM_SET_CURRENT = 0x01
    COMM_SET_CURRENT_BRAKE = 0x02
    COMM_SET_RPM = 0x03
    COMM_SET_POS = 0x04
    COMM_SET_HANDBRAKE = 0x05
    COMM_SET_DETECT = 0x06
    COMM_SET_SERVO_POS = 0x07
    COMM_ALIVE = 0x3A

    def __init__(self, name: str):
        super().__init__(name)
        self.serial_port = None
        self.port = None
        self.baudrate = 115200
        self.timeout = 1.0
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
        self._ramp_up_rate = 0.1  # Power change per second
        self._current_ramp_power = 0.0
        self._direction_change_in_progress = False  # Add direction change lock

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
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both required and optional)
        """
        if self.serial_port:
            self.serial_port.close()
        
        # Get configuration using struct_to_dict for cleaner access
        attributes = struct_to_dict(config.attributes)
        
        # Set up VESC configuration with defaults
        self.port = str(attributes.get("port", "/dev/ttyACM0"))
        self.baudrate = int(attributes.get("baudrate", 115200))
        self.timeout = float(attributes.get("timeout", 1.0))
        self._debug_mode = bool(attributes.get("debug", True))
        self._command_interval = float(attributes.get("command_interval", 0.01))  # Configurable command interval
        self._ramp_up_enabled = bool(attributes.get("ramp_up_enabled", True))
        self._ramp_up_rate = float(attributes.get("ramp_up_rate", 0.1))
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.logger.info(f"Connected to VESC on {self.port} at {self.baudrate} baud")
            
            # Test connection
            if self._test_connection():
                self.logger.info("VESC connection test successful")
            else:
                self.logger.warning("VESC connection test failed - motor may not respond")
                
        except Exception as e:
            self.logger.error(f"Failed to connect to VESC: {e}")
            raise

    def _test_connection(self) -> bool:
        """Test basic VESC connection"""
        try:
            # Try to get VESC values
            if self._send_simple_command(self.COMM_GET_VALUES):
                time.sleep(0.1)
                response = self._read_response()
                return response is not None
            return False
        except Exception as e:
            self.logger.error(f"Connection test failed: {e}")
            return False

    def _crc16(self, data: bytes) -> int:
        """Calculate CRC16 for VESC packet"""
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
        """Pack payload with VESC protocol framing"""
        if len(payload) <= 256:
            header = struct.pack('BB', 2, len(payload))  # Short packet
        else:
            header = struct.pack('BBB', 3, len(payload) >> 8, len(payload) & 0xFF)  # Long packet
        
        crc = self._crc16(payload)
        crc_bytes = struct.pack('>H', crc)
        packet = header + payload + crc_bytes + b'\x03'
        return packet

    def _send_packet(self, command_id: int, payload: bytes = b'') -> bool:
        """Send a packet to the VESC using the working protocol"""
        if not self.serial_port or not self.serial_port.is_open:
            self.logger.error("Serial port not open")
            return False
        
        try:
            # Clear input buffer before sending
            self.serial_port.reset_input_buffer()
            
            # Create the full payload with command ID
            full_payload = struct.pack('B', command_id) + payload
            packet = self._pack_payload(full_payload)
            
            if self._debug_mode:
                self.logger.debug(f"Sending packet: command_id={command_id}, payload={payload.hex()}, full_packet={packet.hex()}")
            
            # Clear output buffer and send
            self.serial_port.reset_output_buffer()
            self.serial_port.write(packet)
            self.serial_port.flush()
            
            # Small delay to ensure command is processed
            time.sleep(0.001)
            
            return True
        except Exception as e:
            self.logger.error(f"Failed to send packet: {e}")
            return False

    def _send_simple_command(self, command_id: int, payload: bytes = b'') -> bool:
        """Send a simple VESC command with basic packet structure"""
        if not self.serial_port or not self.serial_port.is_open:
            self.logger.error("Serial port not open")
            return False
        
        try:
            # Simple packet structure: [START][LEN][PAYLOAD][CRC][END]
            # START = 0x02, END = 0x03
            packet = struct.pack('B', 0x02)  # START
            packet += struct.pack('B', len(payload) + 1)  # LEN (including command ID)
            packet += struct.pack('B', command_id)  # Command ID
            packet += payload  # Payload
            
            # Calculate CRC
            crc_data = packet[1:]  # Everything after START
            crc = self._crc16(crc_data)
            packet += struct.pack('>H', crc)
            packet += struct.pack('B', 0x03)  # END
            
            if self._debug_mode:
                self.logger.debug(f"Sending packet: {packet.hex()}")
            
            self.serial_port.write(packet)
            self.serial_port.flush()
            return True
        except Exception as e:
            self.logger.error(f"Failed to send command: {e}")
            return False

    def _read_response(self, timeout: float = 0.1) -> Optional[bytes]:
        """Read response from VESC with timeout"""
        if not self.serial_port or not self.serial_port.is_open:
            return None
        
        try:
            # Set temporary timeout
            original_timeout = self.serial_port.timeout
            self.serial_port.timeout = timeout
            
            # Read START byte
            start = self.serial_port.read(1)
            if not start or start[0] != 0x02:
                self.serial_port.timeout = original_timeout
                return None
            
            # Read length
            length_bytes = self.serial_port.read(1)
            if not length_bytes:
                self.serial_port.timeout = original_timeout
                return None
            
            length = length_bytes[0]
            
            # Read payload
            payload = self.serial_port.read(length)
            if len(payload) != length:
                self.serial_port.timeout = original_timeout
                return None
            
            # Read CRC
            crc_bytes = self.serial_port.read(2)
            if len(crc_bytes) != 2:
                self.serial_port.timeout = original_timeout
                return None
            
            # Read END byte
            end = self.serial_port.read(1)
            if not end or end[0] != 0x03:
                self.serial_port.timeout = original_timeout
                return None
            
            # Verify CRC
            received_crc = struct.unpack('>H', crc_bytes)[0]
            calculated_crc = self._crc16(struct.pack('B', length) + payload)
            
            if received_crc != calculated_crc:
                self.logger.error(f"CRC mismatch: received {received_crc}, calculated {calculated_crc}")
                self.serial_port.timeout = original_timeout
                return None
            
            # Restore original timeout
            self.serial_port.timeout = original_timeout
            
            if self._debug_mode:
                self.logger.debug(f"Received response: {payload.hex()}")
            
            return payload
            
        except Exception as e:
            self.logger.error(f"Failed to read response: {e}")
            return None

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
        async with self._lock:
            power = max(-1.0, min(1.0, power))  # Clamp to [-1, 1]
            
            # Stop any existing power task
            if self._power_task and not self._power_task.done():
                self._stop_power_task = True
                try:
                    await asyncio.wait_for(self._power_task, timeout=0.5)
                except asyncio.TimeoutError:
                    pass
                self._power_task = None
            
            # Update state
            self._target_power = power
            self._is_powered = power != 0.0
            self._current_power = power
            self._is_moving = power != 0.0
            
            # Send command directly - no complex logic
            duty_float = power
            payload = struct.pack('>f', duty_float)
            self._send_packet(5, payload)  # COMM_SET_DUTY = 5
            
            if power != 0.0:
                # Start simple timer task to keep motor running
                self._stop_power_task = False
                self._power_task = asyncio.create_task(self._simple_power_task())
                self.logger.info(f"Set power to {power:.2f}")
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
        async with self._lock:
            if revolutions == 0:
                await self.stop()
                return
            
            # Try RPM control
            success = False
            try:
                payload = struct.pack('>i', int(rpm))
                if self._send_simple_command(self.COMM_SET_RPM, payload):
                    success = True
                    self.logger.info(f"Set RPM to {rpm}")
            except Exception as e:
                self.logger.debug(f"RPM control failed: {e}")
            
            if success:
                self._current_rpm = rpm
                self._is_powered = True
                self._is_moving = True
                
                # Calculate time needed for the revolutions
                if rpm != 0:
                    time_needed = abs(revolutions / rpm) * 60.0  # Convert to seconds
                    
                    # Wait for the specified time
                    await asyncio.sleep(time_needed)
                    
                    # Stop the motor
                    await self.stop()
                    
                    # Update position
                    self._position += revolutions
            else:
                # Fallback to power control
                self.logger.warning("RPM control failed, using power control fallback")
                power = 0.5 if rpm > 0 else -0.5
                await self.set_power(power)
                await asyncio.sleep(2.0)  # Run for 2 seconds
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
                payload = struct.pack('>i', int(rpm))
                if self._send_simple_command(self.COMM_SET_RPM, payload):
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
            self._position = offset
            self.logger.info(f"Reset zero position with offset {offset}")

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> float:
        """Get the current position in revolutions"""
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
        async with self._lock:
            # Stop background power task
            if self._power_task and not self._power_task.done():
                self._stop_power_task = True
                try:
                    await asyncio.wait_for(self._power_task, timeout=0.5)
                except asyncio.TimeoutError:
                    pass
                self._power_task = None
            
            # Send direct stop command
            duty_float = 0.0
            payload = struct.pack('>f', duty_float)
            self._send_packet(5, payload)  # COMM_SET_DUTY = 5
            
            # Update state
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
        
        if command_name == "get_vesc_values":
            # Get VESC telemetry values
            if self._send_simple_command(self.COMM_GET_VALUES):
                payload = self._read_response()
                if payload:
                    return {"status": "success", "data": payload.hex()}
                else:
                    return {"status": "error", "message": "Failed to read VESC values"}
            else:
                return {"status": "error", "message": "Failed to request VESC values"}
        
        elif command_name == "set_current":
            # Set motor current
            current = command.get("current", 0.0)
            if isinstance(current, (int, float)):
                payload = struct.pack('>f', float(current))
                if self._send_simple_command(self.COMM_SET_CURRENT, payload):
                    return {"status": "success", "current": current}
                else:
                    return {"status": "error", "message": "Failed to set current"}
            else:
                return {"status": "error", "message": "Invalid current value"}
        
        elif command_name == "test_connection":
            # Test VESC connection
            if self._test_connection():
                return {"status": "success", "message": "VESC connection test passed"}
            else:
                return {"status": "error", "message": "VESC connection test failed"}
        
        elif command_name == "ping":
            # Send ALIVE command
            if self._send_simple_command(self.COMM_ALIVE):
                payload = self._read_response()
                if payload:
                    return {"status": "success", "response": payload.hex()}
                else:
                    return {"status": "error", "message": "No response to ping"}
            else:
                return {"status": "error", "message": "Failed to send ping"}
        
        elif command_name == "set_debug":
            # Enable/disable debug mode
            debug = command.get("debug", False)
            self._debug_mode = bool(debug)
            return {"status": "success", "debug_mode": self._debug_mode}
        
        elif command_name == "get_status":
            # Get current motor status
            return {
                "status": "success",
                "is_powered": self._is_powered,
                "current_power": self._current_power,
                "current_rpm": self._current_rpm,
                "is_moving": self._is_moving,
                "position": self._position,
                "debug_mode": self._debug_mode
            }
        
        elif command_name == "set_command_interval":
            # Set command interval for testing different frequencies
            interval = command.get("interval", 0.01)
            if isinstance(interval, (int, float)) and 0.001 <= interval <= 0.1:
                self._command_interval = float(interval)
                return {"status": "success", "command_interval": self._command_interval}
            else:
                return {"status": "error", "message": "Invalid interval (must be between 0.001 and 0.1 seconds)"}
        
        elif command_name == "set_ramp_up":
            # Enable/disable ramp-up
            enabled = command.get("enabled", True)
            self._ramp_up_enabled = bool(enabled)
            return {"status": "success", "ramp_up_enabled": self._ramp_up_enabled}
        
        elif command_name == "set_ramp_rate":
            # Set ramp-up rate
            rate = command.get("rate", 0.1)
            if isinstance(rate, (int, float)) and 0.01 <= rate <= 1.0:
                self._ramp_up_rate = float(rate)
                return {"status": "success", "ramp_up_rate": self._ramp_up_rate}
            else:
                return {"status": "error", "message": "Invalid rate (must be between 0.01 and 1.0 power/second)"}
        
        elif command_name == "get_ramp_status":
            # Get current ramp-up status
            return {
                "status": "success",
                "ramp_up_enabled": self._ramp_up_enabled,
                "ramp_up_rate": self._ramp_up_rate,
                "current_ramp_power": self._current_ramp_power,
                "target_power": self._target_power
            }
        
        elif command_name == "force_stop":
            # Force stop the motor and reset all state
            async with self._lock:
                # Stop power task
                if self._power_task and not self._power_task.done():
                    self._stop_power_task = True
                    try:
                        await asyncio.wait_for(self._power_task, timeout=0.5)
                    except asyncio.TimeoutError:
                        pass
                    self._power_task = None
                
                # Send multiple stop commands
                for _ in range(5):
                    duty_float = 0.0
                    payload = struct.pack('>f', duty_float)
                    self._send_packet(5, payload)  # COMM_SET_DUTY = 5
                    await asyncio.sleep(0.02)
                
                # Reset all state
                self._target_power = 0.0
                self._is_powered = False
                self._current_power = 0.0
                self._current_rpm = 0.0
                self._is_moving = False
                self._current_ramp_power = 0.0
                
                return {"status": "success", "message": "Motor force stopped and state reset"}
        
        elif command_name == "clear_buffers":
            # Clear all serial buffers
            if self.serial_port:
                self.serial_port.reset_input_buffer()
                self.serial_port.reset_output_buffer()
                return {"status": "success", "message": "Serial buffers cleared"}
            else:
                return {"status": "error", "message": "Serial port not open"}
        
        else:
            return {"status": "error", "message": f"Unknown command: {command_name}"}

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> List[Geometry]:
        """Get the geometries of the motor"""
        return []

    async def _simple_power_task(self):
        """Simple timer task to keep motor running"""
        while not self._stop_power_task:
            try:
                # Send command directly - no complex logic
                duty_float = self._target_power
                payload = struct.pack('>f', duty_float)
                self._send_packet(5, payload)  # COMM_SET_DUTY = 5
                await asyncio.sleep(self._command_interval)  # 10ms interval for smoother operation under load
            except Exception as e:
                self.logger.error(f"Simple power task error: {e}")
                await asyncio.sleep(self._command_interval)

