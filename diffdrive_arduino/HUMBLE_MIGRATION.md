# ROS2 Jazzy → Humble Rollback Guide

This file documents the changes required to port the project from ROS2 **Jazzy** back to **Humble**.
Bug fixes (encoder atomicity, safe serial reads, fullnav localization, etc.) apply to both versions — they do not need to be reverted.

---

## 1. Hardware Interface API (`hardware/diffbot_system.hpp` + `hardware/diffbot_system.cpp`)

The hardware interface methods changed in Jazzy (ros2_control 4.x).

### `diffbot_system.hpp`

```cpp
// JAZZY (current):
std::vector<hardware_interface::StateInterface::ConstSharedPtr> on_export_state_interfaces() override;
std::vector<hardware_interface::CommandInterface::SharedPtr>    on_export_command_interfaces() override;

// HUMBLE rollback:
std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
```

### `diffbot_system.cpp`

```cpp
// JAZZY (current):
std::vector<hardware_interface::StateInterface::ConstSharedPtr>
DiffDriveArduinoHardware::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> state_interfaces;
  state_interfaces.push_back(std::make_shared<hardware_interface::StateInterface>(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  // ...
  return state_interfaces;
}

// HUMBLE rollback:
std::vector<hardware_interface::StateInterface>
DiffDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.emplace_back(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos);
  // ...
  return state_interfaces;
}
```

Apply the same transformation for `on_export_command_interfaces` → `export_command_interfaces`:

```cpp
// JAZZY (current):
std::vector<hardware_interface::CommandInterface::SharedPtr>
DiffDriveArduinoHardware::on_export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface::SharedPtr> command_interfaces;
  command_interfaces.push_back(std::make_shared<hardware_interface::CommandInterface>(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));
  // ...
  return command_interfaces;
}

// HUMBLE rollback:
std::vector<hardware_interface::CommandInterface>
DiffDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.emplace_back(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd);
  // ...
  return command_interfaces;
}
```

---

## 2. `bringup/launch/diffbot.launch.py` — `robot_description` parameter

In Jazzy, `ros2_control_node` reads the robot description from the `/robot_description` topic.
In Humble, it must be passed as a parameter.

```python
# JAZZY (current) — control_node:
control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_controllers],
    ...
)

# HUMBLE rollback — also pass robot_description:
control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    parameters=[robot_description, robot_controllers],
    ...
)
```

---

## 3. `bringup/config/navigate_w_replanning_and_recovery.xml` — BT XML node name

Jazzy uses BehaviorTree.CPP **v4**, which removes `SequenceStar`.
Humble uses BT.CPP **v3**, where `SequenceStar` is still valid.

```xml
<!-- JAZZY (current): -->
<Sequence memory="true" name="RecoveryActions">
  ...
</Sequence>

<!-- HUMBLE rollback: -->
<SequenceStar name="RecoveryActions">
  ...
</SequenceStar>
```

---

## 4. `bringup/launch/navigation.launch.py` — BT XML parameter name

```python
# JAZZY (current):
parameters=[configured_params, {'default_nav_to_pose_bt_xml': bt_xml_file}]

# HUMBLE rollback:
parameters=[configured_params, {'default_bt_xml_filename': bt_xml_file}]
```

---

## 5. `bringup/config/nav2_params.yaml` — AMCL motion model name

```yaml
# JAZZY (current):
robot_model_type: "nav2_amcl::DifferentialMotionModel"

# HUMBLE rollback:
robot_model_type: "differential"
```

---

## 6. `bringup/launch/joystick.launch.py` — `twist_stamper` (TODO)

In Jazzy, `diff_drive_controller` only accepts `TwistStamped`.
In Humble, `Twist` is accepted when `use_stamped_vel: false` is set — `twist_stamper` is not needed.

```python
# JAZZY — twist_stamper required (currently a TODO, not yet added)
# HUMBLE rollback — joystick.launch.py stays unchanged; the following
# diffbot_controllers.yaml setting is sufficient:
```

`bringup/config/diffbot_controllers.yaml`:
```yaml
# Valid in Humble, deprecated in Jazzy:
use_stamped_vel: false
```

And in `bringup/launch/diffbot.launch.py`, the `control_node` remappings:
```python
# In Humble, forward Twist via cmd_vel_unstamped:
remappings=[
    ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
]
```

---

## 7. `package.xml` — ROS distribution dependencies

Update package names from `jazzy` → `humble` where they differ:

```xml
<!-- Only update if the package name differs; most are the same -->
<!-- Example: nav2_behaviors in Jazzy was called nav2_recoveries in older distros -->
<exec_depend>nav2_behaviors</exec_depend>  <!-- valid in both -->
```

Check for missing packages with rosdep:
```bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
```

---

## 8. `CMakeLists.txt` — Compiler flags (optional)

Humble is fully compatible with C++17. No changes needed.

---

## Summary

| File | Jazzy (current) | Humble (rollback) |
|---|---|---|
| `diffbot_system.hpp` | `on_export_state_interfaces()` → `ConstSharedPtr` | `export_state_interfaces()` → `StateInterface` |
| `diffbot_system.cpp` | `make_shared<StateInterface>()` | `emplace_back(StateInterface(...))` |
| `diffbot.launch.py` | `parameters=[robot_controllers]` | `parameters=[robot_description, robot_controllers]` |
| `navigate_w_replanning_and_recovery.xml` | `<Sequence memory="true">` | `<SequenceStar>` |
| `navigation.launch.py` | `default_nav_to_pose_bt_xml` | `default_bt_xml_filename` |
| `nav2_params.yaml` (AMCL) | `nav2_amcl::DifferentialMotionModel` | `differential` |
| `diffbot_controllers.yaml` | `use_stamped_vel` can be removed | `use_stamped_vel: false` |
| `diffbot.launch.py` remap | dead code | `cmd_vel_unstamped → /cmd_vel` active |

---

<div align="center">

**MIT License © 2026 Adil NAS**

[![GitHub](https://img.shields.io/badge/GitHub-Adilnasceng-181717?style=flat-square&logo=github)](https://github.com/Adilnasceng)
[![Sponsor](https://img.shields.io/badge/Sponsor-%E2%9D%A4-EC4899?style=flat-square)](https://github.com/sponsors/Adilnasceng)

</div>
