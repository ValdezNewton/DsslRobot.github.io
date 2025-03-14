# Can not Boot Gazebo
**Version**: Ignition Gazebo Fortress, Ros2 Humble, Ubuntu 22.04

## Bug Observation
While launching Gazebo via launch file or simply type 
```bash
ign gazebo
```
and you 
1. See Ignition Gazebo window **Keeps black** 
2. See **messgaes in terminal saying [ros_gz_sim]: requesting list of world names.**

And this lasts forever.


## Solution 

https://github.com/gazebosim/gz-sim/issues/38#issuecomment-2381067266

and disbale the ufw of the device via

```bash
sudo ufw disable
```
