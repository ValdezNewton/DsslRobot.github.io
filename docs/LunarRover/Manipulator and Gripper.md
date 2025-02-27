---
last_update:
  date: 2/27/2025
  author: Bingqi
---

# Manipulator and Gripper

**MAKE SURE YOU STRICTLY FOLLOW THE INSTRUCTIONS IN [`docs/LunarRover/Hardware Assembling and Debugging.md/Ethernet Connection with Realman Robot Manipulator`](./Hardware%20Assembling%20and%20Debugging.md)**



## Manipulator Usage

After connected the manipulator correctly according to [`docs.Hardware Assembling and Debugging`](./Hardware%20Assembling%20and%20Debugging.md), press the power button on both the 24V power cell and the manipulator itself. 

- Seeing the LED **flashing blue first and then green** means the manipulator is powered on and successfully initialized.

To run the manipulator without chassis, run:
```bash
ros2 launch rm_bringup rm_bringup.launch.py
```
Now you're able to see the rviz2 is up and correct joints states displayed. 

IF NOT, YOU'RE FUCKED BY REALMAN :P

## Gripper Usage

**BRING UP THE MANIPULATOR FIRST**. 

Then you should see a topic named `/rm_driver/set_tool_voltage_cmd`, which is used to set the voltage of the I/O at the frange of the manipulator.
```bash
ros2 topic pub --once /rm_driver/set_tool_voltage_cmd std_msgs/msg/UInt16 "{data: 2}"
```
Data `2` means 12V. And now you should see the gripper fingers **opened** and indicating gripper is ready.


**Gripper allow 3 action modes**

1. Close with a fixed speed and **stopped at the target force threshold**. Current mode accepts 3 parameters: 

| Parameter | Description | Value Range | Notes |
|-----------|-------------|-------------|--------|
| `speed` | The speed of closing. | 1~1000 | Dimensionless |
| `force` | The target force threshold. | 1~1000 | Every 1 value represents 0.0015kg |
| `block` | Block or not? | true or false |  |

Example command:
```bash
ros2 topic pub --once /rm_driver/set_gripper_pick_cmd rm_ros_interfaces/msg/Gripperpick "{speed: 200, force: 200, block: true}"
```

Subscribe for command result, and value could be true or false, **error code will printed in manipulator driver terminal**
```bash
ros2 topic echo /rm_driver/set_gripper_pick_result
```


2. Close with a fixed speed, **stopped at the target force threshold**. After stopped, **IF THE FORCE BELOW THRESHOLD AGAIN**, it will close continuously untile the force is above the threshold. Current mode accepts 3 parameters: 

| Parameter | Description | Value Range | Notes |
|-----------|-------------|-------------|--------|
| `speed` | The speed of closing. | 1~1000 | Dimensionless |
| `force` | The target force threshold. | 1~1000 | Every 1 value represents 0.0015kg |
| `block` | Block or not? | true or false |  |

Example command:
```bash
ros2 topic pub --once /rm_driver/set_gripper_pick_on_cmd rm_ros_interfaces/msg/Gripperpick "{speed: 200, force: 200, block: false}"
```

Subscribe for command result, and value could be true or false, **error code will printed in manipulator driver terminal**
```bash
ros2 topic echo /rm_driver/set_gripper_pick_on_result
```

3. Action to target position with a fixed speed. Current mode accepts 2 parameters: 

| Parameter | Description | Value Range | Notes |
|-----------|-------------|-------------|--------|
| `position` | Target position. | 1~1000 | 1~0mm, 1000~70mm |
| `block` | Block or not? | true or false |  |

Example command:
```bash
ros2 topic pub --once /rm_driver/set_gripper_position_cmd rm_ros_interfaces/msg/Gripperset "{position: 200, block: true}"
```

Subscribe for command result, and value could be true or false, **error code will printed in manipulator driver terminal**
```bash
ros2 topic echo /rm_driver/set_gripper_position_result
```
