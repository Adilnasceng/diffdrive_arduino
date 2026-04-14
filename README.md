<div align="center">

# diffdrive_arduino

> ROS2 Humble differential drive robot hardware interface — motor and encoder control via Arduino serial communication.

![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)
![ros2_control](https://img.shields.io/badge/ros2__control-2.x-green)
![License](https://img.shields.io/badge/License-Apache--2.0-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)

</div>

---

## Features

| Feature | Description |
|---|---|
| **Hardware Interface** | Hardware layer compatible with `ros2_control` SystemInterface (Humble / 2.x) |
| **Serial Communication** | Encoder reading and motor control via Arduino using LibSerial |
| **PID Support** | PID values are read from URDF parameters and forwarded to the Arduino |
| **Buzzer Control** | Reverse detection and manual buzzer management |
| **Nav2 Integration** | SLAM, AMCL, EKF and full navigation stack support |

---

## Requirements

**System packages:**

```bash
sudo apt install libserial-dev
```

**ROS2 Humble packages:**

```bash
sudo apt install \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-robot-state-publisher \
  ros-humble-diff-drive-controller \
  ros-humble-joint-state-broadcaster \
  ros-humble-nav2-bringup \
  ros-humble-slam-toolbox \
  ros-humble-robot-localization \
  ros-humble-rplidar-ros \
  ros-humble-joy \
  ros-humble-teleop-twist-joy
```

---

## Installation

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Copy the package
cp -r /path/to/diffdrive_arduino .

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select diffdrive_arduino --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo

# Source the environment
source install/setup.bash
```

---

## Usage

### Robot Bringup

```bash
ros2 launch diffdrive_arduino diffbot.launch.py
```

### Navigation

```bash
# Build a map with SLAM
ros2 launch diffdrive_arduino slam.launch.py

# Localize with a pre-built map
ros2 launch diffdrive_arduino localization.launch.py map:=/path/to/map.yaml

# Full navigation stack
ros2 launch diffdrive_arduino navigation.launch.py
```

### Joystick Control

```bash
ros2 launch diffdrive_arduino joystick.launch.py
```

### Mapping (SLAM + Joystick)

```bash
ros2 launch diffdrive_arduino mapping.launch.py serial_port:=/dev/ttyUSB0
```

### Full Autonomous Navigation (Pre-built Map)

```bash
ros2 launch diffdrive_arduino fullnav.launch.py \
  map:=/home/ubuntu/maps/my_map.yaml \
  serial_port:=/dev/ttyUSB0
```

---

## Verification

After a successful build, verify the system with the following steps:

```bash
# 1. Start bringup
ros2 launch diffdrive_arduino diffbot.launch.py

# 2. Check topics
ros2 topic list | grep -E "cmd_vel|odom|joint_states"

# 3. Monitor odometry data
ros2 topic echo /diffbot_base_controller/odom

# 4. Check joint states
ros2 topic echo /joint_states

# 5. Check controller status
ros2 control list_controllers

# 6. Hardware interface status
ros2 control list_hardware_interfaces
```

Expected controller output:
```
joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
diffbot_base_controller[diff_drive_controller/DiffDriveController] active
```

---

## File Structure

```
diffdrive_arduino/
├── hardware/
│   ├── diffbot_system.cpp          # Hardware interface implementation
│   └── include/diffdrive_arduino/
│       ├── diffbot_system.hpp      # Hardware interface header
│       ├── arduino_comms.hpp       # Serial communication (LibSerial)
│       └── wheel.hpp               # Wheel kinematic model
├── description/
│   ├── urdf/                       # URDF/xacro robot description
│   ├── ros2_control/               # ros2_control plugin configuration
│   └── launch/                     # RViz2 visualization
├── bringup/
│   ├── launch/                     # All launch files
│   └── config/                     # YAML configuration files
├── diffdrive_arduino.xml           # pluginlib definition
├── CMakeLists.txt
└── package.xml
```

---

## Tech Stack

- **ros2_control 2.x** — Humble-compatible hardware interface (SystemInterface)
- **LibSerial** — Serial port communication with Arduino
- **Nav2** — Navigation, SLAM, AMCL
- **robot_localization** — Odometry fusion with EKF
- **C++17** — Modern C++ standard

---

## TODO

> Known missing features not yet implemented.

- [ ] **twist_stamper integration** — In Jazzy, `diff_drive_controller` expects `TwistStamped`. Since `teleop_twist_joy` and Nav2 velocity smoother publish `Twist`, the `twist_stamper` node should be added to `diffbot.launch.py`, the commented line in `joystick.launch.py` should be removed, and the input remap updated to `/cmd_vel_in → /cmd_vel`. If joystick control currently works, the `use_stamped_vel: false` parameter is still valid; this step becomes mandatory if that parameter is removed in Jazzy.

---

## Migration Note (Humble → Jazzy)

This package was originally developed for ROS2 Humble. Key API changes when migrating to Jazzy:

| Change | Humble | Jazzy |
|---|---|---|
| State interface export | `export_state_interfaces()` → `vector<StateInterface>` | `on_export_state_interfaces()` → `vector<StateInterface::ConstSharedPtr>` |
| Command interface export | `export_command_interfaces()` → `vector<CommandInterface>` | `on_export_command_interfaces()` → `vector<CommandInterface::SharedPtr>` |
| ros2_control_node | `robot_description` as parameter | Read via topic |

---

<div align="center">

**MIT License © 2026 Adil NAS**

[![GitHub](https://img.shields.io/badge/GitHub-Adilnasceng-181717?style=flat-square&logo=github)](https://github.com/Adilnasceng)
[![Sponsor](https://img.shields.io/badge/Sponsor-%E2%9D%A4-EC4899?style=flat-square)](https://github.com/sponsors/Adilnasceng)

</div>
