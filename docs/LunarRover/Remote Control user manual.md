---
last_update:
  date: 09/02/2025
  author: Isaac Hiew
---

# Remote Control

## Introduction
The Remote Control system allows operators to control the entire LunarBot — including chassis mobility and manipulator arm — using an XBOX 360 wireless joystick.

It integrates:
- Chassis control (via `vos_bringup`)
- Manipulator control (via rm_bringup)
- Gripper operations (via message bridge + rm_driver)
- Sensor visualization (via sensor_hardware)

Remote control requires both hardware (joystick, batteries, receiver) and software (ROS 2 launch files) to be properly configured.

## Prerequisites
### Hardware 
- Two 24V batteries powering chassis and manipulator.
- Mini PC installed onboard LunarBot.
- XBOX 360 wireless joystick + USB receiver.

### Software
- ROS 2 Humble installed.
- Lunar_Rover_Hardware_Ws workspace with all hardware packages:
    - vos_bringup (chassis)
    - rm_bringup (manipulator)
    - lunar_rover_remote (joystick interface)
    - message_bridge (gripper + trajectory bridging)
    - sensor_hardware (LiDAR integration, optional for visualization).

### Network Setup
- Connect controller PC to WiFi hotspot:
    - SSID: REDMI
    - Password: dssl123456
- Remote desktop access via TEAMVIEWER:
    - IP: 192.168.1.145
    - Password: dssl62794316

## Remote Control Workflow
### Step 1 – Power On
1. Switch on both 24V batteries.
2. Power up robotic arm.
3. Power up chassis system.

### Step 2 – Joystick Setup
1. Plug in XBOX 360 USB receiver.
2. Switch on joystick; it must indicate connected to receiver 1.

### Step 3 – Remote Access
1. From control PC, launch TEAMVIEWER.
2. Connect to LunarBot mini PC using provided IP and password.

### Step 4 – Launch Control Nodes
Inside LunarBot’s remote desktop, open four terminals and run:

**Terminal 1 – Joystick driver**
```bash
sudo xboxdrv --silent
```
Must confirm joystick connected without errors, if not replug receiver and restart joystick.

**Terminal 2 – Chassis bringup**
```bash
Copy code
cd Lunar_Rover_Hardware_Ws
. ./install/setup.bash
ros2 launch vos_bringup vos_bringup.launch
```
Must print **VOS_CONTROLLER IS ACTIVE**, if not restart LunarBot and Mini PC.

**Terminal 3 – Manipulator bringup**
```bash
Copy code
cd Lunar_Rover_Hardware_Ws
. ./install/setup.bash
ros2 launch rm_bringup rm_75_bringup.launch.py
```
Launches `rm_driver`, `rm_description`, `rm_control`, and MoveIt2 config.

**Terminal 4 – Remote control mapping**
```bash
Copy code
cd Lunar_Rover_Hardware_Ws
. ./install/setup.bash
ros2 launch lunar_rover_remote remote.launch.py
```
This node maps joystick axes and buttons to LunarBot motion.

### Step 5 – Operation
- Press button B + Joystick → Control chassis movement.
- Joystick only → Control arm manipulator.
- Gripper commands bridged via gripper_bridge_node (from message_bridge).

## Package Integration 
- `vos_bringup` → Initializes chassis hardware controllers
- `rm_bringup` → Initializes manipulator arm, description, and drivers
- `message_bridge` → Bridges joystick gripper commands to rm_driver
- `lunar_rover_remote` → Reads joystick input and publishes commands to arm & chassis.
- `sensor_hardware` → Optional LiDAR visualization in RViz

## Troubleshooting
### Joystick Issues
- If sudo xboxdrv reports no device:
    - Replug USB receiver.
    - Restart joystick.
- If joystick not controlling LunarBot:
    - In lunar_rover_remote launch files, check device ID.
    - Different USB ports may assign different device IDs

### Chassis/Manipulator Not Responding
- Ensure both batteries are powered.
- Restart vos_bringup and rm_bringup.

### Gripper Not Moving
- Check if rm_driver is running.
- Echo topic:
```bash
ros2 topic echo /rm_driver/set_gripper_position_cmd
```
- If empty, restart gripper_bridge_node.

### TEAMVIEWER Connection Fails
- Verify WiFi connection to REDMI hotspot.
- Ping 192.168.1.145 from control PC.

## Recommended Workflow
1. Power on chasssis + arm
2. Connect joystick (check `xboxdrv`)
3. Start bringup:
    - Chassis (vos_bringup)
    - Arm (rm_bringup)
    - Remote control (lunar_rover_remote)
4. Drive LunarBot using joystick
5. Plan motions with MoveIt2 if needed ( via `rm_moveit2_config`)
6. Use LiDAR (`sensor_hardware`) for obstacle detection in RViz.

## Safety Notes
- Always test joystick mapping in an open space.
- Emergency stop: power off both 24V batteries.
- Avoid rapid joystick switching, which may overload controller nodes.
