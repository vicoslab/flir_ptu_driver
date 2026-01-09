# FLIR PTU Driver

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

A basic ROS One driver for FLIR E-series Pan-Tilt units, based on [FLIR-PTU-Python](https://github.com/hmorris94/FLIR-PTU-Python).

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
roslaunch flir_ptu_driver ptu.launch
```

Or with run:
```bash
rosrun flir_ptu_driver ptu_node.py
```

## UDEV Rules (optional)

To make the serial port consistent, we can map it to a specific device based on its parameters.


Find the `ID_VENDOR_ID` and `ID_MODEL_ID` with:
```bash
udevadm info -q all -n /dev/ttyUSB0
```


Then create the rule:
```bash
sudo nano /etc/udev/rules.d/99-pan-tilt.rules
```

And add:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="0557", ATTRS{idProduct}=="2008", SYMLINK+="pantilt", MODE="0666"
```

Reload:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

We should now have a symlink from `/dev/ttyUSBX` -> `/dev/pantilt`.