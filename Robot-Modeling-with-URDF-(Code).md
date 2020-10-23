###### tags: `ROS` `URDF` `xacro`

Robot Modeling with URDF (Code)
===
Unified Robot Description Format (**URDF**) is used in ROS to describe robot models.
URDF describes the relation between components.
Here we explain how to describe a robot with URDF.
#### References
[ROS wiki urdf](http://wiki.ros.org/urdf/Tutorials)

# Overview
URDF mainly consists of two components:
- **Link**: A robot component which does not move.
- **Joint**: A connection between links.

<!-- ![](https://i.imgur.com/eqQrrdc.png =300x) -->
<img src="https://i.imgur.com/eqQrrdc.png" width=300/>

Links are connected by joints.
Each joint has a parent link and a child link and describes the position of a child relative to its parent.

# Example
Here is an example of a URDF file describing a simple robot.
Add this file as `<your robot description pkg>/urdf/myrobot.urdf`.
```=xml
<?xml version="1.0"?>
<robot name="materials">

  <material name="blue">
    <color rgba="0 0 0.8 1"/>
  </material>

  <material name="white">
    <color rgba="1 1 1 1"/>
  </material>


  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.6" radius="0.2"/>
      </geometry>
      <material name="blue"/>
    </visual>
  </link>

  <link name="right_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_right_leg" type="fixed">
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="0 -0.22 0.25"/>
  </joint>

  <link name="left_leg">
    <visual>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
      <origin rpy="0 1.57075 0" xyz="0 0 -0.3"/>
      <material name="white"/>
    </visual>
  </link>

  <joint name="base_to_left_leg" type="fixed">
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0 0.22 0.25"/>
  </joint>
```

## Visualize URDF in RViz
To visualize your robot, you can use [urdf_tutorial](http://wiki.ros.org/urdf_tutorial) package.
You can install it with a following command:

    $ sudo apt-get install ros-melodic-urdf-tutorial

You can see the model in RViz by launching the `dislplay.launch` file:

    $ roslaunch urdf_tutorial displlay.launch model:=urdf/myrobot.urdf
    


<img src="https://i.imgur.com/XMVPhP8.png" width=300/>


# XML specifications
You can learn more about XML specifications in [ROS wiki](http://wiki.ros.org/urdf/XML)

URDF is an XML format.
This is the approximate structure of URDF:

- robot
    - link
        - inertial
            - origin
            - mass
            - inertia
        - visual
            - origin
            - **geometry**
                - box
                - cylinder
                - sphere
                - mesh
            - material
                - color
                - texture
        - collision
            - origin
            - geometry
    - joint
        - origin
        - **parent**
        - **child**
        - axis
        - calibration
        - dynamics
        - limit
        - mimic
        - safety_controller
    - transmission
    - gazebo

