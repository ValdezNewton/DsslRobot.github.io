---
last_update:
  date: 09/01/2025
  author: Bingqi
  co-author: Isaac Hiew
---

# LunarBot Hardware Introduction
A lunar rover is a robotic vehicle designed to traverse across the rocky surface of the Moon. It plays a crucial role in space exploration by collecting data, transporting equipment and conducting experiments far beyond the landing site of a spacecraft. 

The LunarBot is the **fundamental** unit of a lunar service robot cluster, developed by the Distributed and Intelligent Space System Lab (DSSL) at Tsinghua University. It is an uncrewed robotic rover designed to collect data, take photos of the lunar terrain. It is built on a **4-wheeled independent steering and driving platform**, each wheel features a driving motor and steering motor with encoders. It is equipped with a multi-modal perception system that integrates a Livox-Mid360 360° LiDAR, a Intel Realsense D435i camera and IMU integrated in it. Realman RM-75, a 7-degrees-of-freedom (7-DOF) robotic manipulator, with a 2-finger gripper as end effector, is also integrated.

![LunarBot](../../static/img/LunarBot.jpeg)

## Geometric Information
The dimensions of the LunarBot is approximately 1303.05mm length, 997.09mm width and 782mm height without solar pannels and manipulator. Its wheels feature a $diameter, \Phi= 260mm $ (without EVA foam) and $width = 168mm$.

## Components

Current LunarBot is a 4 wheel model.

### Wheels
The LunarBot shares four identical wheels, each is equipped with the following:
  * A 100 Watts steering motor with encoder
  * A 200 Watts driving motor with encoder
  * 1 origin indicator sensor (npn switch)
  * 1 limitaion indicator sensor (npn switch)
  * 2 servos, for each motor
  * Steering and driving motor gear box

Except servos are installed in chassis body, all other components are exposed at each wheel.

### Chamber
Inside the chamber of the chassis, we got:
  * A high-voltage graphene power cell
  * A low-voltage graphene power cell
  * 8 servos
  * Circuit breakers
  * Computation Module

### Sensors
  * Livox-Mid360 LiDAR
  * Intel Realsense D435i Camera for 3D vision
  * IMU

### Manipulators
  * Realman RM-75 robot manipupator, 7-dof
  * Inspire Robots EG2-4C gripper
