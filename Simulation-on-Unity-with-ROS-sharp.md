###### tags: `ROS` `Modeling` `ROS#` `Unity`

ROS Simulation on Unity with ROS#
===
[ROS#](https://github.com/siemens/ros-sharp/) is a library for communicating with ROS from Unity.

# Install
You can set up ROS# according to [ROS# Wiki](https://github.com/siemens/ros-sharp/wiki).
Windows and Oracle VM are used as platforms of Unity and ROS respectively in this Wiki, but you can use Unity on Mac OS and ROS on another computer. 

First, clone or download [ros-sharp](https://github.com/siemens/ros-sharp.git) folder.

## Setting up Unity
[ROS# Wiki Unity on Windows](https://github.com/siemens/ros-sharp/wiki/User_Inst_Unity3DOnWindows)
1. Create a new Unity project.
2. Copy **RosSharp** folder (ros-sharp/Unity3D/Assets/RosSharp) into the **Assets** folder of your Unity project.
3. Make sure that Unity is using **.NET Framework 4.x**:
    - `Edit > Project Settings > Player > Other Settings > Configuration`
    - You can find .NET settings (Its menu screen is different by OS).

## Setting up ROS
1. Install `rosbridge-suite` via
```
$ sudo apt-get install ros-melodic-rosbridge-server
```
2. Copy **file_server** folder (ros-sharp/ROS/file_server) into your `src` folder of your Catkin workspace.
3. Run `$ catkin_make` in your Catkin workspace.

# Simulation Example
We use this [wheel robot model](https://sites.google.com/site/robotlabo/time-tracker/ros/gazebo-mobilerobot) as an exapmle URDF model.
This robot model uses **xacro** format to describe a robot, but you cannot use xacro format in Unity so firstly you have to generate URDF file from xacro format:

    $ rosrun xacro xacro wheel_robot_base.urdf.xacro > wheel_robot.urdf
    
Then, copy/move `wheel_robot.urdf` into `Assets/Urdf` folder in your Unity project.

## Setup Scene (in Unity)
Floor and GameObject are the minimum required.
First, place a floor via `GameObject > 3D Object > Plane`.


### Place URDF object
Click `GameObject > 3D Object > URDF Model (import)` and select `Assets/Urdf/wheel_robot.urdf`.

You can see like this:

<img src='https://i.imgur.com/wl85t39.png' width=500/>


## TwistSubscriber
Attach **TwistSubscriber** (Assets/RosSharp/Scripts/RosBridgeClient/RosCommunication/TwistSubscriber.cs) to `base_link` (`wheel_robot_base > base_footprint > base_link`)
(`base_link` is the main object in URDF model).

Then, open `Inspector` window of `base_link`.

### Ros Connector
Set `Ros Bridge Server Url` in the tab `Ros Connector (Script)` .
The format of`Ros Bridge Server Url` is `<IP address of your ROS server>:9090(default)`

### Twist Subscriber
In the `Twist Subscriber (Script)` set as the table below:



| Attribute |  |
| --- | ----|
| Topic     | `/cmd_vel` |
| Subscribed Trasform | `base_link` |


<img src=https://i.imgur.com/W7816rM.png width=400/>


## ROS setting
In the ROS, run these two commands:
1. `$ roslaunch rosbridge_server rosbridge_websocket.launch`
2. `$ rqt`

Then **rqt** window will pop up.
Select `Plugins > Robot Tools > Robot Steering`.
Set the Topic `/cmd_vel`.
<img src=https://i.imgur.com/1xBjmUq.png width=500/>


## Simulation
The preparation is completed.
Push the **Play** button <img src=https://i.imgur.com/jZtcjYy.png/> and change the values of Robot Steering.






