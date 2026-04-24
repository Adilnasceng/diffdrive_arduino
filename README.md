<div align="center">

# diffdrive_arduino

> ROS2 differential drive robot hardware interface — motor and encoder control via serial communication with full Nav2 integration.

![ROS2](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue)
![Hardware](https://img.shields.io/badge/Hardware-Arduino%20%7C%20STM32F407-red)
![ros2_control](https://img.shields.io/badge/ros2__control-2.x%20%7C%204.x-green)
![License](https://img.shields.io/badge/License-Apache--2.0-blue)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)

</div>

---

## Branches

Each branch is a self-contained variant for a specific ROS2 distribution and hardware target:

| Branch | ROS2 | ros2_control | Microcontroller |
|---|---|---|---|
| [`humble`](../../tree/humble) | Humble | 2.x | Arduino Mega |
| [`jazzy`](../../tree/jazzy) | Jazzy | 4.x | Arduino Mega |
| [`humble-stm32`](../../tree/humble-stm32) | Humble | 2.x | STM32F407G-DISC1 |
| [`stmf407`](../../tree/stmf407) | Jazzy | 4.x | STM32F407G-DISC1 |

Clone the branch that matches your ROS2 distribution and hardware:

```bash
# Arduino Mega + ROS2 Humble
git clone -b humble https://github.com/Adilnasceng/diffdrive_arduino.git

# Arduino Mega + ROS2 Jazzy
git clone -b jazzy https://github.com/Adilnasceng/diffdrive_arduino.git

# STM32F407G-DISC1 + ROS2 Humble
git clone -b humble-stm32 https://github.com/Adilnasceng/diffdrive_arduino.git

# STM32F407G-DISC1 + ROS2 Jazzy
git clone -b stmf407 https://github.com/Adilnasceng/diffdrive_arduino.git
```

---

## Features

| Feature | Description |
|---|---|
| **Hardware Interface** | `ros2_control` SystemInterface hardware layer |
| **Serial Communication** | Encoder reading and motor control via LibSerial |
| **PID Support** | PID values read from URDF parameters and forwarded to the microcontroller |
| **Buzzer Control** | Reverse detection and manual buzzer management |
| **Nav2 Integration** | SLAM, AMCL, EKF and full navigation stack support |

---

## Quick Start

```bash
# Create workspace
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src

# Clone the branch matching your setup
# Example: STM32F407G-DISC1 + ROS2 Jazzy
git clone -b stmf407 https://github.com/Adilnasceng/diffdrive_arduino.git

# Install dependencies and build
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select diffdrive_arduino --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

See the **branch README** for detailed installation, usage, and hardware wiring instructions.

---

<div align="center">

**MIT License © 2026 Adil NAS**

[![GitHub](https://img.shields.io/badge/GitHub-Adilnasceng-181717?style=flat-square&logo=github)](https://github.com/Adilnasceng)
[![Sponsor](https://img.shields.io/badge/Sponsor-%E2%9D%A4-EC4899?style=flat-square)](https://github.com/sponsors/Adilnasceng)

</div>

