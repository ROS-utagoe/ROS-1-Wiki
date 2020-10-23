###### tags: `ROS`

Comparison of Simulation Tools
===



|  | Gazebo | Unity | Unreal Engine |
| -------- | -------- | -------- |---|
| Topic Comm| **A+** | **A** [ROS#](https://github.com/siemens/ros-sharp) | **A** [ROSIntegration](https://github.com/code-iai/ROSIntegration) or [UROSBridge](https://github.com/robcog-iai/UROSBridge) |
| Using URDF | **A** | **A** | **C** (usable with [plugin](https://github.com/robcog-iai/URoboSim ) but deprecated)
| Modeling | **A** (SDF) | **A** (URDF) | not try |
| Convert to URDF | [pysdf](http://wiki.ros.org/pysdf): SDF to URDF | [ROS#](https://github.com/siemens/ros-sharp) | --
| Materials | **B** [Model Database](https://bitbucket.org/osrf/gazebo_models/src/default/) | **A** Asset Store | **A** Marketplace|
| Sensors | **A** [Plugins](http://gazebosim.org/tutorials?tut=ros_gzplugins) |

# 3D Model Format

## Correspondence Table
| 3D Format | ROS(RViz) | Gazebo | Unity | Unreal Engine |
| -------- | -------- | -------- |---|----|
| STL | :heavy_check_mark:(only binary file) | :heavy_check_mark: | :heavy_check_mark:(ROS#) | :x: |
| COLLADA | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: | :x: |
| FBX | :x: | :x: | :heavy_check_mark: | :heavy_check_mark: |
| OBJ | :x: | :heavy_check_mark: | :heavy_check_mark: | :x: |

## 3D Format
| Format | Texture | Features |
| -------- | -------- | -------- |
| STL     | :x:    | often used with 3D printer |
| COLLADA | :heavy_check_mark: | all objects |
| OBJ | :heavy_check_mark: | static object |
| FBX | :heavy_check_mark: | all objects |

- It seems STL format is inferior to other formats, but its data is very small so it is smooth to simulate it.
- FBX is often used in 3D Game Engine.
