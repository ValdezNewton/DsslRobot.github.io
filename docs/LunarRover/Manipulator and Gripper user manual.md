---
last_update:
  date: 09/02/2025
  author: Isaac Hiew
---

# Manipulator and Gripper

## Overview
The Manipulator and Gripper system equips the Lunar Rover with dexterous manipulation capability.
It consists of:
- Manipulator (arm): A Realman robotic arm, connected via Ethernet and powered with a 24V supply.
- Gripper: An electrically actuated end-effector connected to the manipulator flange and controlled through ROS 2 nodes.
This manual explains how to assemble, power up, and operate both the manipulator and gripper in simulation and real hardware.

## Safety Precautions
- Always verify power connections before turning on the system
- Ensure clear workspace around the manipulator before operation
- Keep personnel at a safe distance during manipulator movement
- Follow all electrical safety protocols

**NOTE: ENSURE YOU STRICTLY FOLLOW THE INSTRUCTIONS IN [`docs/LunarRover/Hardware Assembling and Debugging.md/Ethernet Connection with Realman Robot Manipulator`](./Hardware%20Assembling%20and%20Debugging.md)** 

## Prerequisites
### Dependencies
- Serial Library: The gripper driver depends on the serial library. Since ROS 2 does not support Catkin-based builds, clone the following repo:
```bash
git clone https://gitee.com/laiguanren/serial.git
```
- CH340 USB to RS485 driver: Install drivers from CH341SER_LINUX to connect the gripper to /dev/ttyCH341USB0.

### Common Issues
- If /dev/ttyCH341USB0 is missing:
```bash
sudo apt remove brltty
```
Then replug the dongle.
- If you get serial unable to open:
```bash
sudo chmod 777 /dev/ttyCH341USB0
```

## System Initialization
### Power on Sequence
1. Connect the manipulator according to **Hardware Assembling and Debugging**
2. Press the power button on the 24V power cell
3. Press the power button on the manipulator itself
4. Observe LED status:
    - Flashing blue first and then green: Successful initialization
    - Other patterns: Refer to hardware troubleshooting guide

### Software Launch
To run the manipulator without chassis:
```bash
ros2 launch rm_bringup rm_bringup.launch.py
```

Verification: Check that rviz2 launches successfully and displays correct joint states.

## Gripper Operation
### Power Activation
Prerequisite: The manipulator must be brought up first.
Enable gripper power (12V output):
```bash
ros2 topic pub --once /rm_driver/set_tool_voltage_cmd std_msgs/msg/UInt16 "{data: 2}"
```
Expected Response: Gripper fingers should open, indicating readiness.

## Gripper Action Modes
### Mode 1: Close with Force Threshold
Closes at fixed speed and stops when target force is reached.
**Parameters:**
| Parameter | Description | Value Range | Notes |
|-----------|-------------|-------------|--------|
| `speed` | Closing speed | 1~1000 | Dimensionless |
| `force` | Target force threshold | 1~1000 | 1 value = 0.0015kg |
| `block` | Blocking mode | true/false |  |

**Command:**
```bash
ros2 topic pub --once /rm_driver/set_gripper_pick_cmd rm_ros_interfaces/msg/Gripperpick "{speed: 200, force: 200, block: true}"
```

**Result Monitoring:**
```bash
ros2 topic echo /rm_driver/set_gripper_pick_result
```

### Mode 2: Adaptive Close with Force Monitoring
Closes at fixed speed, stops at force threshold, and resumes if force drops below threshold.
**Parameters: Same as Mode 1**

**Command:**
```bash
ros2 topic pub --once /rm_driver/set_gripper_pick_on_cmd rm_ros_interfaces/msg/Gripperpick "{speed: 200, force: 200, block: false}"
```

**Result Monitoring:**
```bash
ros2 topic echo /rm_driver/set_gripper_pick_on_result
```

### Mode 3: Position Control
Moves to target position at fixed speed.

**Parameters:**

| Parameter | Description | Value Range | Notes |
|-----------|-------------|-------------|--------|
| `position` | Target position. | 1~1000 | 1 refers to 0mm, 1000 refers to 70mm |
| `block` | Block or not? | true or false |  |

**Command:**
```bash
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 200, block: true}"
```

**Result Monitoring:**
```bash
ros2 topic echo /rm_driver/set_gripper_position_result
```

## Low-level Gripper Driver (Serial Node)
The Gripper_control node (`Gripper_control.cpp`) provides service-based control for advanced scenarios.

**Launch:**
```bash
ros2 run inspire_gripper Gripper_control_node
```

**Available Services:**
| Service          | Function                                |
| ---------------- | --------------------------------------- |
| `SetID`          | Assign a new gripper ID                 |
| `Setopenlimit`   | Set max/min opening limits              |
| `Setmovetgt`     | Move to target opening width            |
| `Setmovemax`     | Release at set speed                    |
| `Setmovemin`     | Close with speed + force                |
| `Setmoveminhold` | Close continuously with force threshold |
| `Setestop`       | Emergency stop                          |
| `Setparam`       | Save parameters permanently             |
| `Getopenlimit`   | Query opening limits                    |
| `Getcopen`       | Get current opening width               |
| `Getstatus`      | Get status, error code, temperature     |

This allows deeper integration and diagnostic access compared to the high-level Realman driver.

## Recommended Workflow
1. Assemble and power on manipulator + gripper.
2. Launch manipulator:
```bash
ros2 launch rm_bringup rm_bringup.launch.py
```
3. Enable tool voltage (12V):
```bash
ros2 topic pub --once /rm_driver/set_tool_voltage_cmd std_msgs/msg/UInt16 "{data: 2}"
```
4. Control gripper via:
- ROS 2 topics (for high-level pick/place operations).
- Gripper_control services (for fine-grained control and debugging).

## Notes & Troubleshooting
- Always bring up manipulator before powering gripper.
- For serial driver errors: check port /dev/ttyCH341USB0 and permissions.
- Use Getstatus service to debug errors and overheating issues.
- Emergency stop (Setestop) is available for safety-critical operations.
