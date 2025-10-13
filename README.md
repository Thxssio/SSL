# PyDynamixel V2 - TauraBots Edition

Small helpers for controlling Dynamixel MX series (Protocol 2.0) on Linux
systems such as Raspberry Pi.

Author: Mauricio Moraes Godoy  
Date: 2025-09-15

## Features

- Focused on Dynamixel Protocol 2.0 (e.g., MX-28, MX-64, MX-106)
- Simple classes for position mode (`Joint`) and velocity mode (`Wheel`)
- Broadcast torque control and sync read/write utilities via `DxlCommV2`
- Clean NumPy-style docstrings across the codebase

## Installation

1. Install the official Dynamixel SDK for Python.
   Refer to Robotis docs or simply:

```bash
pip install dynamixel-sdk
```

2. Copy or install this package to your project.

## Quick Start

```python
from ssl_raspberry import DxlCommV2, Joint, Wheel, constants

# 1) Open port
comm = DxlCommV2(port="/dev/ttyUSB0", baudrate=57600)

# 2) Create devices
shoulder = Joint(servo_id=1, center_value=2048)
wheel = Wheel(servo_id=2)

# 3) Attach to the communication manager
comm.attach(shoulder)
comm.attach(wheel)

# 4) Enable torque (broadcast)
comm.enable_all_torque()

# 5) Command
shoulder.send_angle(45.0)        # degrees
wheel.send_velocity(30.0)        # rpm

# 6) Read back positions
positions = comm.read_positions()
print(positions)
```

## UDP Protocol (drive command)

Control is done via JSON over UDP. The recommended single message format groups
all relevant fields in one packet while there is no IMU on-board.

- Port: default `20011` (configurable via `--udp-port`)
- Units:
  - **vx, vy**: meters per second (m/s), robot frame if no `heading`, or field
    frame if `heading` is provided (radians)
  - **vtheta**: radians per second (rad/s)
  - **theta**: current robot heading in radians (optional)
  - **theta_target**: desired heading in radians (optional)

Message examples:

```json
{"cmd":"drive","vx":0.2,"vy":0.0,"vtheta":0.0,"theta":null,"theta_target":null}
```

```json
{"cmd":"drive","vx":0.0,"vy":0.0,"vtheta":0.5,"theta":1.57,"theta_target":3.14}
```

Notes:
- When `theta` is present, it updates the internal heading estimate.
- When `theta_target` is present, a proportional controller adjusts `vtheta`.
- If you want to command in the field frame, use the legacy `vel` with
  `heading` (in radians) or extend `drive` to include `heading` as needed.

Kinematics reference: see RoboTeam Twente's omnidirectional control write-up
which follows Rojas' formulation. Wheel speeds are computed in rad/s and
mapped to Dynamixel RPM for commands, with the inverse applied for feedback
paths. Refer to the “Wheels to feedback” section for the full chain between
wheel angular speeds and robot velocities.

- Reference: [RoboTeam Twente – Omnidirectional Control](https://wiki.roboteamtwente.nl/technical/control/omnidirectional#wheels_to_feedback)

## Units and conversions

| Quantity                     | Symbol          | Unit   | Notes |
| ---------------------------- | --------------- | ------ | ----- |
| Linear velocity (x, y)       | vx, vy          | m/s    | Robot frame by default; field frame if `heading` provided |
| Angular velocity (robot)     | vtheta          | rad/s  | Positive CCW |
| Current heading              | theta           | rad    | From IMU/vision when available |
| Desired heading              | theta_target    | rad    | Proportional correction applied in controller |
| Wheel angular speed (kin)    | ω_wheel         | rad/s  | Output of kinematics; used internally |
| Wheel command to Dynamixel   | RPM             | RPM    | Converted from rad/s for SDK command |

Conversions:

```text
RPM = (rad/s) * 60 / (2π)
rad/s = RPM * (2π / 60)
```

In code, we compute wheel speeds in rad/s from robot (vx, vy, vtheta), then map
to RPM for the Dynamixel command. Feedback is read in RPM and converted back to
rad/s before applying the inverse kinematics to reconstruct (vx, vy, vtheta).

## API Overview

- `BaseDynamixel`: Common fields and helpers.
- `Joint`:
  - `set_goal_angle(angle, radian=False)`
  - `send_angle(angle, radian=False)`
  - `get_angle(radian=False) -> float`
- `Wheel`:
  - `set_goal_velocity(rpm)`
  - `send_velocity(rpm)`
  - `get_velocity() -> float`
- `DxlCommV2`:
  - `attach(dxl)`
  - `enable_all_torque()` / `disable_all_torque()`
  - `send_positions({id: degrees})`
  - `read_positions() -> {id: degrees}`

## Notes

- Angles are mapped with a nominal scale of 2048 ≈ 180 degrees on MX series.
- Velocity scale uses 0.229 factor (SDK convention) from internal unit to RPM.
- This project is intentionally minimal and pragmatic for quick experiments.

## License

MIT