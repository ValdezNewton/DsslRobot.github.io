---
last_update:
  date: 2/25/2025
  author: Bingqi
---

# LunarBot Hardware Assembling and Debugging

**This is not a step-by-step tutorial but will definately help you avoid stupid mistakes that you might led to fatal errors and bugs!!**

To assemble the LunarBot from sketch, you need a **chassis body**, **wheels**, **electro board** with servos and other essential components on board, **powering system**, **sensors** that could be customized as you wish, and a **computation module** that could be a nuc running Ubuntu22.04 with enough I/O.

## Wheels to Servos
To connect wheels to servos, we need to take 4 powering cables including U,V,W and PE, 2 powering cables for sensors to $ 24V^+$ and $24V^-$ (or GND), and a signal cable to servo.

Just plug all powering cables from motors to correspond servos following lables on the servo. 

**PAY ATTENTION** to powering cables from origin and limitation sensors from wheels. All $ 24V^+$ cable should be plugged in powering module on electro board, and all $24V^-$ should be **connected to SAME GND RELATIVE TO $24V^+$ of each steering servo**.

Signal cable for origin and limitation sensors could be pluged in multiple "signal-in" slots on **driving servos**, as long as you configure **proper signal-in slot in kinco software for the servo driver**. 

## Servo to Computation Module
All servo should be in series circuit connect by RJ45 cables from "in" to "out" one to another. Last servo in this series provide a RJ45 cable that used to communicate with computation module. Use a certified RJ45 extractor to get CAN_H and CAN_L lines, and then through a USB-CAN converter is an elegant way to let software aware of these cute wheels and servos.

**!!!ATTENTION!!!**

* Look up documents about servos which lines of RJ45 is CAN_H and CAN_L. This must match RJ$% extractor and later match the USB_CAN device.
* **REMEMBER TO SETUP CAN PORTS IN COMPUTATION MODULE BY RUNNNING FOLLOWING COMMANDS**
```bash
sudo ip link set can0 txqueuelen 1000
sudo ip link set can0 up type can bitrate 500000
```

Substitute "can0" with your actural device and correct bitrate.

## Powering Module BUGS

When the auto-initialize process seems in-correct that **wheel is not rotating smoothly**, or **servos keeps powering off automatically** just after powered on, then **IT"S TIME TO JUICE UP THE BATTERY**. Don't believe the capacity indicator on the battery!

