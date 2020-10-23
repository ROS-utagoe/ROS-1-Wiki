# Navigation with rviz on Ubuntu
You can also navigate with RViz on Ubuntu.



# Tank Setup
Set up Tank before navigation.
It is almost the same from that of [Android App](Tank-slam-nav-android.md).

## 1. IMU Calibration
Touch IMU calibration command and calibrate IMU sensors.
Wait a while until calibration is completed.

<img src="fig/touchscreen-menu-1.jpg" width=400>

You'll see this window when calibration is done.

<img src="fig/touchscreen-menu-2.jpg" width=400>


## 2. Make a Map
Next, push `make a map` command and move the tank around.
You can use `mouse_teleop` to drive your Tank.
### Install `mouse_teleop` package
```
$ sudo apt install ros-melodic-mouse-teleop
```
### Run `mouse_teleop`
The `mouse_teleop` node publishes its message to the topic `mouse_vel` by default.
You have to remap topic name to `cmd_vel`.
```
$ rosrun mouse_teleop mouse_teleop.py mouse_vel:=cmd_vel
```

## 3. Save a Map
Push `save map` command after moving around.

## 4. Navigation
Push `navigation` command at last.