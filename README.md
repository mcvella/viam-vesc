# VESC Motor Controller Module

A Viam module for controlling VESC (Vedder Electronic Speed Controller) motor controllers, specifically designed for the Flipsky FSESC 4.12 and compatible VESC units.

## Features

- **Direct Serial Communication**: Uses direct serial commands for reliable communication
- **Motor Control**: Full motor control including power, RPM, position, and current control
- **Telemetry**: Access to VESC telemetry data
- **Safety Features**: Proper CRC checking and error handling
- **Async Support**: Full async/await support for non-blocking operations

## Model mcvella:vesc:vesc

A motor component that interfaces with VESC motor controllers via serial communication.

### Configuration

The following attribute template can be used to configure this model:

```json
{
  "port": "/dev/ttyUSB0",
  "baudrate": 115200,
  "timeout": 1.0
}
```

#### Attributes

The following attributes are available for this model:

| Name       | Type   | Inclusion | Default        | Description                                    |
|------------|--------|-----------|----------------|------------------------------------------------|
| `port`     | string | Optional  | "/dev/ttyACM0" | Serial port path (see port detection guide below) |
| `baudrate` | int    | Optional  | 115200         | Serial communication baudrate                  |
| `timeout`  | float  | Optional  | 1.0            | Serial communication timeout in seconds        |

#### Port Detection

To find the correct serial port for your VESC:

**Linux:**
```bash
ls /dev/ttyACM*
ls /dev/ttyUSB*
```

**macOS:**
```bash
ls /dev/tty.usbserial*
ls /dev/tty.usbmodem*
```

**Windows:**
Check Device Manager under "Ports (COM & LPT)" for devices like "USB Serial Device (COM3)"

Common port patterns:
- Linux: `/dev/ttyACM0`, `/dev/ttyUSB0`
- macOS: `/dev/tty.usbserial-0001`, `/dev/tty.usbmodem14101`
- Windows: `COM3`, `COM4`

#### Example Configuration

```json
{
  "name": "vesc_motor",
  "type": "motor",
  "model": "mcvella:vesc:vesc",
  "attributes": {
    "port": "/dev/ttyUSB0",
    "baudrate": 115200,
    "timeout": 1.0
  }
}
```

### Supported Operations

#### Basic Motor Control

- `set_power(power)`: Set motor power as percentage (-1.0 to 1.0)
- `set_rpm(rpm)`: Set motor RPM
- `go_for(rpm, revolutions)`: Run motor for specific revolutions at given RPM
- `go_to(rpm, position)`: Go to specific position at given RPM
- `stop()`: Stop the motor
- `reset_zero_position(offset)`: Reset zero position with offset

#### Status Queries

- `get_position()`: Get current position in revolutions
- `is_powered()`: Check if motor is powered and get current power
- `is_moving()`: Check if motor is moving
- `get_properties()`: Get motor properties

### DoCommand

The VESC motor supports custom commands for advanced functionality:

#### Get VESC Telemetry

```json
{
  "command": "get_vesc_values"
}
```

Returns VESC telemetry data including voltage, current, temperature, RPM, etc.

#### Set Motor Current

```json
{
  "command": "set_current",
  "current": 10.5
}
```

Sets the motor current in amperes (positive for forward, negative for reverse).

### Hardware Setup

#### Flipsky FSESC 4.12

1. **Power Connection**: Connect battery to VESC power terminals
2. **Motor Connection**: Connect motor phases to VESC motor terminals
3. **Serial Connection**: Connect USB-to-serial adapter to VESC UART pins
4. **Configuration**: Use VESC Tool to configure motor parameters

#### Serial Pinout

- **TX**: Connect to VESC RX pin
- **RX**: Connect to VESC TX pin
- **GND**: Connect to VESC GND pin

### Troubleshooting

#### Common Issues

1. **Connection Failed**: Check serial port path and permissions
2. **No Response**: Verify baudrate matches VESC configuration
3. **CRC Errors**: Check wiring and ensure proper voltage levels
4. **Motor Not Moving**: Verify motor connections and VESC configuration

#### Debug Mode

Enable debug logging by running viam-server with the `--debug` flag:

```bash
viam-server --debug
```

### Safety Notes

- Always ensure proper motor configuration in VESC Tool before use
- Monitor motor temperature and current during operation
- Use appropriate current limits for your motor
- Ensure proper power supply voltage and current capacity

### License

This module is licensed under the same license as the Viam SDK.
