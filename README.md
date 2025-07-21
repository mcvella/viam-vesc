# VESC Motor Controller Module

A Viam module for controlling VESC (Vedder Electronic Speed Controller) motor controllers, compatible with all VESC-based controllers. Tested with Flipsky FSESC 4.12, 75_300_R2, and other VESC hardware.

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

## Configuration Options

- `duty_cycle_format` (optional):
    - `int` (default): Use 32-bit integer for duty cycle (scaled by 100,000). Required for modern VESC firmware (e.g., 75_300_R2, VESC 6/7, FW 5.2+).
    - `float`: Use 32-bit float for duty cycle (range -1.0 to 1.0). For legacy VESC firmware that expects float payloads.

**Example:**
```json
{
  "attributes": {
    "port": "/dev/ttyACM0",
    "duty_cycle_format": "float"  // Use only if your VESC firmware requires float
  }
}
```

If you are unsure, leave this option out (default is `int`).

### Supported Operations

#### Basic Motor Control

- `

## Ramp-Up / Ramp-Down Behavior

By default, the module ramps motor power smoothly to the target value instead of changing instantly. This is controlled by two parameters:

- `ramp_up_enabled` (default: true): Enables ramping behavior.
- `ramp_up_rate` (default: 0.1): Maximum change in power per second (e.g., 0.1 means it takes 10 seconds to go from 0 to 1.0 power).
- `command_interval` (default: 0.01): How often the power is updated (in seconds).

**Ramp Time Examples (with default settings):**

| Target Power | Time to Reach |
|--------------|--------------|
| 1.0          | 10 seconds   |
| 0.5          | 5 seconds    |
| 0.2          | 2 seconds    |
| 0.1          | 1 second     |

**To make ramping faster, increase `ramp_up_rate` in your config.**

**Example for 1 second ramp to full power:**
```json
{
  "ramp_up_rate": 1.0,
  "command_interval": 0.01
}
```

Set `ramp_up_enabled` to `false` to disable ramping and change power instantly.