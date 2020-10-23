# Twist Publisher with Dualshock 4
Almost all robots subscribe `/cmd_vel` topic whose message type is `Twist` for controlling robots.

Here we demonstrate how to publish Twist message to `/cmd_vel` using [**Dualshock 4**](https://www.playstation.com/ja-jp/explore/accessories/gaming-controllers/dualshock-4/) which is an official controller of [PlayStation 4](https://www.playstation.com/ja-jp/explore/ps4/).

<img src="fig/dualshock4.jpeg">

### Required ROS Packges (installed by default)
- joy
- [teleop_twist_joy](https://github.com/ros-teleop/teleop_twist_joy)

### Optional
- [ds4drv](https://github.com/chrippa/ds4drv)

# Edit File
```
- /opt/ros/melodic/share/teleop_twist_joy/
    - config/
        - ds4.config.yaml
    - launch/
        - ds4_teleop.launch
```

#### `ds4.config.yaml`
```yaml
axis_linear: 1
scale_linear: 0.7
scale_linear_turbo: 1.0

axis_angular: 0
scale_angular: 1

enable_button: 1  # L2 shoulder button
enable_turbo_button: 0  # L1 shoulder button
```

#### `ds4_teleop.launch`
```xml
<launch>
  <arg name="joy_config" default="ds4" />
  <arg name="joy_dev" default="/dev/input/js0" />
  <arg name="config_filepath" default="$(find teleop_twist_joy)/config/$(arg joy_config).config.yaml" />
  
  <node pkg="joy" type="joy_node" name="joy_node">
    <param name="dev" value="$(arg joy_dev)" />
    <param name="deadzone" value="0.3" />
    <param name="autorepeat_rate" value="20" />
  </node>

  <node pkg="teleop_twist_joy" name="teleop_twist_joy" type="teleop_node">
    <rosparam command="load" file="$(arg config_filepath)" />
  </node>
</launch>

```