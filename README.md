# VESC Motor Controller Module

A Viam module for controlling VESC (Vedder Electronic Speed Controller) motor controllers, compatible with all VESC-based controllers. Tested with Flipsky FSESC 4.12, 75_300_R2, and other VESC hardware.

## Features

- **Serial or SocketCAN**: Choose UART/USB serial (default) or Linux SocketCAN
- **Multi-motor CAN**: Address multiple VESC controllers on one bus by controller `id`
- **Motor Control**: Full motor control including power, RPM, position, and current control
- **Telemetry**: Access to VESC telemetry data (serial `COMM_GET_VALUES` or CAN status frames)
- **Safety Features**: Proper CRC checking (serial) and error handling
- **Async Support**: Full async/await support for non-blocking operations

## Model mcvella:vesc:vesc

A motor component that interfaces with VESC motor controllers via serial or SocketCAN.

### Configuration

Use `transport` to select the I/O backend (`serial` default, or `can`).

#### Serial (default)

```json
{
  "transport": "serial",
  "port": "/dev/ttyUSB0",
  "baudrate": 115200,
  "timeout": 1.0
}
```

| Name       | Type   | Inclusion | Default        | Description                                    |
|------------|--------|-----------|----------------|------------------------------------------------|
| `transport`| string | Optional  | `"serial"`     | `"serial"` or `"can"`                          |
| `port`     | string | Optional  | "/dev/ttyACM0" | Serial port path (see port detection guide below) |
| `baudrate` | int    | Optional  | 115200         | Serial communication baudrate                  |
| `timeout`  | float  | Optional  | 1.0            | Serial communication timeout in seconds        |

#### SocketCAN

```json
{
  "transport": "can",
  "interface": "can0",
  "id": 1
}
```

| Name        | Type   | Inclusion | Default | Description                                      |
|-------------|--------|-----------|---------|--------------------------------------------------|
| `transport` | string | Required  | —       | Must be `"can"`                                  |
| `interface` | string | Required  | —       | Linux CAN device (e.g. `can0`)                   |
| `id`        | int    | Required  | —       | VESC controller ID on the bus (0–255)            |
| `ticks_per_rotation` | float | Optional | `1.0` | STATUS_5 tachometer counts per revolution for `GetPosition` |

CAN uses 29-bit extended frames with ID `(command << 8) | id`, matching the [VESC CAN protocol](https://github.com/vedderb/bldc/blob/master/documentation/comm_can.md) and the [erh/vesccan](https://github.com/erh/vesccan) reference module.

**Position (CAN):** `GetPosition` uses the STATUS_5 (`0x1B00 \| id`) **tachometer** (signed int32 in bytes 0–3). Revolutions = `(tachometer − zero) / ticks_per_rotation`. `attributes.id` must match the VESC id in that frame. Call `ResetZeroPosition` after startup before measuring travel. DoCommand `{"command":"get_position_debug"}` shows whether STATUS_5 was seen. Serial still uses a software counter.

**SocketCAN is Linux-only.** Bring up the interface before starting the module, for example:

```bash
sudo ip link set can0 type can bitrate 500000
sudo ip link set up can0
```

#### Two motors on one CAN bus

```json
{
  "components": [
    {
      "name": "left",
      "type": "motor",
      "model": "mcvella:vesc:vesc",
      "attributes": {
        "transport": "can",
        "interface": "can0",
        "id": 1
      }
    },
    {
      "name": "right",
      "type": "motor",
      "model": "mcvella:vesc:vesc",
      "attributes": {
        "transport": "can",
        "interface": "can0",
        "id": 2
      }
    }
  ]
}
```

Each component opens its own SocketCAN socket filtered by `id`.

#### Port Detection (serial)

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

#### Example Configuration (serial)

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

- `duty_cycle_format` (optional, serial only):
    - `int` (default): Use 32-bit integer for duty cycle (scaled by 100,000). Required for modern VESC firmware (e.g., 75_300_R2, VESC 6/7, FW 5.2+).
    - `float`: Use 32-bit float for duty cycle (range -1.0 to 1.0). For legacy VESC firmware that expects float payloads.

**Example:**
```json
{
  "attributes": {
    "port": "/dev/ttyACM0",
    "duty_cycle_format": "float"
  }
}
```

If you are unsure, leave this option out (default is `int`). CAN always uses the integer scaled duty format.

## Ramp-Up / Ramp-Down Behavior

By default, the module ramps motor power toward the new target instead of jumping instantly.

| Attribute | Default | Meaning |
|-----------|---------|---------|
| `ramp_up_enabled` | `true` | When `false`, power changes immediately. |
| `ramp_up_rate` | `0.25` | Max change in power **per second** (power is −1.0…1.0). |
| `command_interval` | `0.01` | How often the keepalive/ramp loop ticks (seconds). Does **not** set total ramp time. |

**Time to finish a ramp:**

```text
seconds ≈ |new_power − current_power| / ramp_up_rate
```

With the default `ramp_up_rate` of `0.25`:

| Power change | Approximate time |
|--------------|------------------|
| 0 → 1.0 (full) | 4 s |
| 0 → 0.5 | 2 s |
| −0.5 → 0.5 | 4 s |

Faster ramp → higher `ramp_up_rate`. Slower ramp → lower `ramp_up_rate`.

**Examples:**

```json
{
  "ramp_up_rate": 1.0
}
```
≈ 1 second for a full 0 → 1.0 change.

```json
{
  "ramp_up_rate": 0.2
}
```
≈ 5 seconds for a full 0 → 1.0 change.

```json
{
  "ramp_up_enabled": false
}
```
Disable ramping; apply the new power immediately.

## Publishing (Viam cloud build)

This repo is set up for [Viam cloud build](https://docs.viam.com/build-modules/manage-modules/) via [`.github/workflows/viam-build.yml`](.github/workflows/viam-build.yml).

**One-time setup**

1. Create an org API key: `viam organizations api-key create --org-id <org-id> --name viam-vesc-cloud-build`
2. In the GitHub repo, add Actions secrets:
   - `viam_key_id`
   - `viam_key_value`

**Publish a version**

1. Push your changes, then create a GitHub Release (tag like `0.2.0`).
2. The workflow runs cloud builds for `linux/amd64`, `linux/arm64`, and `darwin/arm64` and uploads them to the registry.

`build.sh` packages the full `python-can` (SocketCAN) backend into the PyInstaller binary. If you see `No module named 'can.interfaces.socketcan'`, publish a new release that includes that change (or point the machine at a local `./run.sh` module after `setup.sh`).

**Local build smoke test**

```sh
viam module build local
```

For local machine testing without packaging, keep using `./run.sh` in your machine config (as in `example_config.json`). Registry uploads use the PyInstaller binary `dist/main`.
