###### tags: `ROS` `Gazebo` `Rviz`

Robot Modeling with Gazebo
===
Gazebo Tutorials: http://gazebosim.org/tutorials

# URDF vs SDF
Unified Robot Description Format (**URDF**) is used in ROS to describe robot models.
URDF describes the relation between components.
On the other hand, Simulation Discription Format (**SDF**) is used in Gazebo because it describes not only objects but also simulation environment.
The difference is explained in detail [here](https://answers.gazebosim.org//question/62/sdf-vs-urdf-what-should-one-use/).
For above reasons, **SDF** is used in Gazebo.

### Convert URDF to SDF
http://gazebosim.org/tutorials?tut=ros_urdf

### Convert SDF to URDF
https://kenbell.hatenablog.com/entry/20161119/1479545879
http://wiki.ros.org/pysdf

# Model Description
## SDFormat
`<joint>` tag reference is [here](http://sdformat.org/spec?ver=1.4&elem=joint#model_joint).
### joint type
(sdf version = 1.5)
| type | Description |
| -------- | -------- |
| revolute    | a hinge joint that rotates on a single axis with either a fixed or continuous range of motion    |
| gearbox | geared revolute joints |
| revolute2 | same as two revolute joints connected in series |
| prismatic | a sliding joint that slides along an axis with a limeted ranve specified by upper and lower limits |
| ball | a ball and socket joint |
| screw | a single degree of freedom joint with coupled sliding and rotational motion|
| universal | like a ball joint but constrains one degree of freedom |
| fixed | a joint with zero degrees of freedom that rigidly connects two links


### Script
```xml
<?xml version='1.0'?>
<sdf version='1.4'>

    <model name="diff_drive">
    <static>false</static>
        <link name='chassis'>
            <pose>0 0 .1 0 0 0</pose>

            <collision name='collision'>
                <geometry>
                <box>
                    <size>.4 .2 .1</size>
                </box>
                </geometry>
            </collision>

            <visual name='visual'>
                <geometry>
                <box>
                    <size>.4 .2 .1</size>
                </box>
                </geometry>
            </visual>

            <collision name='caster_collision'>
                <pose>-0.15 0 -0.05 0 0 0</pose>
                <geometry>
                    <sphere>
                        <radius>.05</radius>
                    </sphere>
                </geometry>

                <surface>
                    <friction>
                        <ode>
                            <mu>0</mu>
                            <mu2>0</mu2>
                            <slip1>1.0</slip1>
                            <slip2>1.0</slip2>
                        </ode>
                    </friction>
                </surface>
             </collision>

            <visual name='caster_visual'>
                <pose>-0.15 0 -0.05 0 0 0</pose>
                <geometry>
                    <sphere>
                        <radius>.05</radius>
                    </sphere>
                </geometry>
            </visual>

        </link>

        <link name="left_wheel">
            <pose>0.1 0.13 0.1 0 1.5707 1.5707</pose>
            <collision name="collision">
            <geometry>
                <cylinder>
                <radius>.1</radius>
                <length>.05</length>
                </cylinder>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <cylinder>
                <radius>.1</radius>
                <length>.05</length>
                </cylinder>
            </geometry>
            </visual>
        </link>

        <link name="right_wheel">
            <pose>0.1 -0.13 0.1 0 1.5707 1.5707</pose>
            <collision name="collision">
            <geometry>
                <cylinder>
                <radius>.1</radius>
                <length>.05</length>
                </cylinder>
            </geometry>
            </collision>
            <visual name="visual">
            <geometry>
                <cylinder>
                <radius>.1</radius>
                <length>.05</length>
                </cylinder>
            </geometry>
            </visual>
        </link>

        <joint type="revolute" name="left_wheel_hinge">
            <pose>0 0 -0.03 0 0 0</pose>
            <child>left_wheel</child>
            <parent>chassis</parent>
            <axis>
            <xyz>0 1 0</xyz>
            </axis>
        </joint>

        <joint type="revolute" name="right_wheel_hinge">
            <pose>0 0 0.03 0 0 0</pose>
            <child>right_wheel</child>
            <parent>chassis</parent>
            <axis>
            <xyz>0 1 0</xyz>
            </axis>
        </joint>

    </model>
</sdf>
```
### Insert the Mode
Select the created model in the `Insert` tab.

![](https://i.imgur.com/NmOrj8m.png)

### Add a Sensor
http://gazebosim.org/tutorials/?tut=add_laser
model repository: https://github.com/osrf/gazebo_models
To add a sensor(e.g. Hokuyo Laser sensor) to the robot, add these lines before `</model>` tag.
```xml
    <include>
      <uri>model://hokuyo</uri>
      <pose>0.2 0 0.2 0 0 0</pose>
    </include>
    <joint name="hokuyo_joint" type="fixed">
      <child>hokuyo::link</child>
      <parent>chassis</parent>
    </joint>
```
![](https://i.imgur.com/FjHGsv7.png)



## Model Editor (GUI Tool)
http://gazebosim.org/tutorials?tut=model_editor&cat=model_editor_top
http://gazebosim.org/tutorials?tut=guided_b3





# Model Database
https://bitbucket.org/osrf/gazebo_models/src/default/
https://github.com/osrf/gazebo_models



# References
[ロボットモデリング講習会(URDF)](https://gbiggs.github.io/rosjp_urdf_tutorial_text/mobile_robot_gazebo.html)

[Qiita: ROS講座13 URDFを記述する1](https://qiita.com/srs/items/35bbaadd6c4be1e39bb9)
