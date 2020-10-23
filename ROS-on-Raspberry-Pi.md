###### tags: `ROS` `Raspberry Pi`

ROS on Raspberry Pi
===
We describe the way of utilizing ROS Melodic on Raspberry Pi 4(Raspbian Buster).
# Installation
You can install ROS Melodic on Raspberry Pi 4 following [this website](http://wiki.ros.org/ROSberryPi/Installing%20ROS%20Melodic%20on%20the%20Raspberry%20Pi).
You might want to write `source /opt/ros/melodic/setup.bash` in `.bashrc`.

# Preparation

You can learn fundamental things about ROS following ROS wiki's [Tutorials](http://wiki.ros.org/ROS/Tutorials).

## Creating a ROS Workspace
Reference: [Installing and Configuring Your ROS Environment](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment)

First, you have to create *workspace*. 
`catkin_ws` is often used as a workspace name.
Then run `catkin_make` in the workspace `~/catkin_ws` to make workspace.
```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin_make
```
After creating workspace, run the following to set your workspace properly overlayed.
```
$ source ~/catkin_ws/devel/setup.bash
```
You have to source this script everytime you set a new terminal.
You might want to write it to `~/.bashrc`.

## Creating a ROS Package
Follow ROS wiki's [Tutorials](http://wiki.ros.org/ROS/Tutorials) if you want to use existing packages.

Follow [Creating an ROS Package](http://wiki.ros.org/ROS/Tutorials/CreatingPackage) if you want to create your own packages.

Create your own package in `~/catkin_ws/src`.

    $ cd ~/catkin_ws/src
    $ catkin_create_pkg <Package Name> std_msgs rospy roscpp
    
You can set a list of dependencis after `<Package Name>`.


# Message (Python)
You can learn more about msg and srv in [ROS wiki: Creating a ROS msg and srv](http://wiki.ros.org/ROS/Tutorials/CreatingMsgAndSrv).

## 1.Create a msg
First, let's create a `*.msg` file.
Let me take an example for message **Motor**:

    $ roscd <Package Name>
    $ mkdir msg
    $ touch msg/Motor.msg

***Motor.msg***
```
float32 direction
float32 speed
```

## 2.Edit CMakeLists.txt
You have to edit `CMakeLists.txt` file to activate your `*.msg` file.

Make sure you add `std_msgs` and `message_generation` to the list of COMPONENTS:
```
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)
```


