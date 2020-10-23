###### tags: `ROS` `Gazebo`

ROS Connection with Gazebo
===

# Create Your own Gazebo Package
http://gazebosim.org/tutorials?tut=ros_roslaunch

First, create `ROBOT_description` package and `ROBOT_gazebo` package using `catkin_create_pkg` command, and create directories as bellow:
```
../catkin_ws/src
    /ROBOT_description
        package.xml
        CMakeLists.txt
        /models
            /ROBOT
                model.config
                model.sdf
        /meshes
            mesh1.dae
            mesh2.dae
            ...
        /materials
        /cad
    /ROBOT_gazebo
        /launch
            ROBOT_sim.launch
        /worlds
            ROBOT_sim.world
        /models
            world_object1.dae
            world_object2.stl
            world_object3.urdf
        /materials
        /plugins
```
## ROBOT_description
In this package describe your robots and place materials.
### models
Put your model files in this directory.
You can use **Model Editor** to create a model.
> **Model Editor**
> http://gazebosim.org/tutorials?tut=model_editor&cat=model_editor_top
http://gazebosim.org/tutorials?tut=guided_b3
```
ROBOT_description/models
    /ROBOT
        model.config
        model.sdf
    /ROBOT_2
        ...
```
Save your robot `model.config` and `model.sdf` like the above.

### Plugin
Add plugins to move your robot.
[Gazebo Plugin in ROS](http://gazebosim.org/tutorials?tut=ros_gzplugins) can be used for sensors and actuators.

Here we use **Differential Drive** plugin.
Add these lines in `model.sdf`.
#### model.sdf
```xml
<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='ROBOT'>

  ...

  <plugin name="differential_drive_controller" filename="libgazebo_ros_diff_drive.so">
        <alwaysOn>true</alwaysOn>
        <updateRate>20</updateRate>
        <leftJoint>base_link_left_wheel</leftJoint>
        <rightJoint>base_link_right_wheel</rightJoint>
        <wheelSeparation>0.26</wheelSeparation>
        <wheelDiameter>0.2</wheelDiameter>
        <wheelAcceleration>1.0</wheelAcceleration>
        <wheelTorque>20</wheelTorque>
        <commandTopic>cmd_vel</commandTopic>
        <robotBaseFrame>chassis</robotBaseFrame>
  </plugin>

  </model>
</sdf>

```
#### Write a Plugin
You can write your own plugin following this tutorial: [Gazebo -- Write a Plugin](http://gazebosim.org/tutorials?cat=write_plugin).

## ROBOT_gazebo
Place materials used in Gazebo in this package.
### worlds
Describe your simulation world.
Here is a simple example.
```xml
<?xml version="1.0" ?>
<sdf version="1.4">
    <world name="default">
        <include>
            <uri>model://ground_plane</uri>
        </include>
        <include>
            <uri>model://sun</uri>
        </include>
        <include>
            <uri>model://gas_station</uri>
            <name>gas_station</name>
            <pose>-2.0 7.0 0 0 0 0</pose>
        </include>
    </world>
</sdf>
```
Gazebo automatically downloads models from [Model Database](http://models.gazebosim.org/).

### launch
Here, write a launch file `ROBOT_sim.launch`
```xml
<launch>
    <include file="$(find gazebo_ros)/launch/empty_world.launch">
        <arg name="world_name" value="$(find ROBOT_gazebo)/worlds/ROBOT_sim.world"/>
    </include>
    <node name="spawn_model" pkg="gazebo_ros" type="spawn_model" args="-file $(find ROBOT_description)/models/ROBOT/model.sdf -sdf -model ROBOT" />
</launch>
```

# Simulation in Gazebo
First, launch with `ROBOT_sim.launch` file:

    $ roslaunch ROBOT_gazebo ROBOT_sim.launch

You can see Gazebo window open and your robot is in it.
![](https://i.imgur.com/2CEyUbl.png)

Next, use **Robot Steering** tool in **rqt** to publish a message to the topic `/cmd_vel`.

Run `$ rqt` and then **rqt** window will pop up.
Select `Plugins > Robot Tools > Robot Steering`.
Set the Topic `/cmd_vel`.
<img src=https://i.imgur.com/1xBjmUq.png width=500/>
