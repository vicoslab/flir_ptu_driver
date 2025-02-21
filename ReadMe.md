# FLIR PTU Driver

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A basic ROS 2 driver for FLIR E-series Pan-Tilt units, based on [FLIR-PTU-Python](https://github.com/hmorris94/FLIR-PTU-Python).

## Parameters

- **port** (`string`, default: `/dev/ttyUSB0`) – Serial port to which the PTU is connected.
- **baud** (`integer`, default: `9600`) – Baud rate for serial communication with the PTU.
- **publishing_rate** (`double`, default: `5.0`) – Frequency (Hz) at which the PTU state is published.

## Topics

- **`ptu/state`** (`sensor_msgs/JointState`) – Publishes the current pan and tilt angles of the PTU in radians.
  - `name` – Joint names (`["ptu_pan", "ptu_tilt"]`).
  - `position` – Current joint positions in radians.

- **`ptu/cmd`** (`sensor_msgs/JointState`) – Receives commands to move the PTU to specified angles.
  - `name` – Expected to contain `"ptu_pan"` and `"ptu_tilt"`.
  - `position` – Desired joint positions in radians.
  - `velocity` (optional) – Desired joint velocities in rad/s.

## Running the Node

```bash
ros2 launch flir_ptu_driver ptu.launch.py
```

Or with run:
```bash
ros2 run flir_ptu_driver ptu_node.py
```