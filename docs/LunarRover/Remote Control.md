---
last_update:
  date: 10/25/2024
  author: CabbageDog
---

# Remote Control

With all hardware assembly, software installation and LAN setup, you can control the entire LunarBot with XBOX 360 joystick. 

## STEP BY STEP
1. Turn on two 24V batteries, robotic arm and chassis.
2. Connect your device to WIFI: REDMI with PASSWORD: dssl123456. 
3. Launch **TEAMVIEWER** app, connect to the LunarBot by ip: 192.168.1.145, password: dssl62794316.
4. Turn on XBOX360 joy stick. it should indicate **connected to receiver 1**. 
4. **WITH IN** the remote desktop of the LunarBot, run following commands
```bash
# Terminal 1
sudo xboxdrv --silent

# IT MUST SAYS CONNECTED WITHOUT ANY ERROR, 
#   IF USB UNCONNECTED, RE-PLUG THE RECEIVER AND RUN AGAIN
#   IF NO XBOX CONTROLLER CONNECTED, RE_PLUG RECEIVER WHILE RESTART THE JOYSTICK, THEN RUN AGAIN
```
```bash
# Terminal 2
cd Lunar_Rover_Hardware_Ws
. ./install/setup.bash
ros2 launch vos_bringup vos_bringup.launch

# IT MUSH SAYS VOS_CONTROLLER IS ACTIVE, IF NOT:
#   RESTART LUNARBOT
#   RESTART MINI PC
```

```bash
# Terminal 3
cd Lunar_Rover_Hardware_Ws
. ./install/setup.bash
ros2 launch rm_bringup rm_75_bringup.launch.py
```

```bash
# Terminal 4
cd Lunar_Rover_Hardware_Ws
. ./install/setup.bash
ros2 launch lunar_rover_remote remote.launch.py
```

6. GOOD TO GO. **PRESS BUTTON B** then stick for chassis and only stick for arm.

## TROUBLESHOOTING

### Config the Correct Device ID of the Joystick
In `lunar_rover_remote` package, all launch file starting `joy_node` should be assigned with correct device ID. 

**Different USB PORT that Receiver of Joystick** may lead to different device id. 

If you found that it's not working when all scripts above is running correctly, check this.