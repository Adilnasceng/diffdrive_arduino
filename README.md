<div align="center">

# diffdrive_arduino

> ROS2 Jazzy differential drive robot hardware interface — motor and encoder control via STM32 serial communication.

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue)
![ros2_control](https://img.shields.io/badge/ros2__control-4.x-green)
![License](https://img.shields.io/badge/License-Apache--2.0-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)

</div>

---

## Features

| Feature | Description |
|---|---|
| **Hardware Interface** | Hardware layer compatible with `ros2_control` SystemInterface (Jazzy / 4.x) |
| **Serial Communication** | Encoder reading and motor control via STM32 using LibSerial |
| **PID Support** | PID values are read from URDF parameters and forwarded to the STM32 |
| **Buzzer Control** | Reverse detection and manual buzzer management |
| **Nav2 Integration** | SLAM, AMCL, EKF and full navigation stack support |

---

## Requirements

**System packages:**

```bash
sudo apt install libserial-dev
```

**ROS2 Jazzy packages:**

```bash
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-diff-drive-controller \
  ros-jazzy-joint-state-broadcaster \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-rplidar-ros \
  ros-jazzy-joy \
  ros-jazzy-teleop-twist-joy
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
ros2 launch diffdrive_arduino mapping.launch.py serial_port:=/dev/ttyACM0
```

### Full Autonomous Navigation (Pre-built Map)

```bash
ros2 launch diffdrive_arduino fullnav.launch.py \
  map:=/home/ubuntu/maps/my_map.yaml \
  serial_port:=/dev/ttyACM0
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

## STM32F407G-DISC1 Pin Configuration

### Arduino IDE Settings

| Setting | Value |
|---|---|
| Board Manager | STM32 MCU based boards (stm32duino) |
| Board | Generic STM32F4 series |
| Board part number | Discovery F407VG |
| USB support | CDC (generic 'Serial' supersede U(S)ART) |
| Upload method | STM32CubeProgrammer (SWD) |

### Encoder Pins — Hardware Timer Mode

| Signal | STM32 Pin | Timer / AF | Description |
|---|---|---|---|
| Left Encoder A | **PA15** | TIM2_CH1 · AF1 | Left wheel A phase (32-bit timer) |
| Left Encoder B | **PB3** | TIM2_CH2 · AF1 | Left wheel B phase (32-bit timer) |
| Right Encoder A | **PB4** | TIM3_CH1 · AF2 | Right wheel A phase (16-bit timer) |
| Right Encoder B | **PB5** | TIM3_CH2 · AF2 | Right wheel B phase (16-bit timer) |

> PA15 = JTDI, PB3 = JTDO — these pins are free when uploading via SWD; not available with JTAG.

### Motor Driver Pins — BTS7960

| Signal | STM32 Pin | Timer / AF | BTS7960 Input |
|---|---|---|---|
| Left Motor Forward | **PE9** | TIM1_CH1 · AF1 | L_PWM |
| Left Motor Reverse | **PE11** | TIM1_CH2 · AF1 | R_PWM |
| Right Motor Forward | **PE13** | TIM1_CH3 · AF1 | L_PWM |
| Right Motor Reverse | **PE14** | TIM1_CH4 · AF1 | R_PWM |

> BTS7960 EN pins are tied to VCC in hardware. PWM frequency: **20 kHz**.

### Other Pins

| Signal | STM32 Pin | Description |
|---|---|---|
| Buzzer | **PE7** | Active buzzer — HIGH = on |
| Serial (USB CDC) | **PA11 / PA12** | Micro-USB USER port — no auto-reset |

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

- **ros2_control 4.x** — Jazzy-compatible hardware interface (SystemInterface)
- **LibSerial** — Serial port communication with STM32
- **Nav2** — Navigation, SLAM, AMCL
- **robot_localization** — Odometry fusion with EKF
- **C++17** — Modern C++ standard

---

## TODO

> Known missing features not yet implemented.

- [ ] **twist_stamper integration** — In Jazzy, `diff_drive_controller` expects `TwistStamped`. Since `teleop_twist_joy` and Nav2 velocity smoother publish `Twist`, the `twist_stamper` node should be added to `diffbot.launch.py`, the commented line in `joystick.launch.py` should be removed, and the input remap updated to `/cmd_vel_in → /cmd_vel`. If joystick control currently works, the `use_stamped_vel: false` parameter is still valid; this step becomes mandatory if that parameter is removed in a future Jazzy release.

---

## Migration Note (Humble → Jazzy)

Key API changes from ROS2 Humble to Jazzy:

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
