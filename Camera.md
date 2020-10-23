# Camera

## Image Publisher
### Required Packages
- [uvc_camera](http://wiki.ros.org/uvc_camera)
- [compressed_image_transport](http://wiki.ros.org/compressed_image_transport)

These packages are initially installed in a ROS Desktop Package(`ros-melodic-desktop` or `ros-melodic-desktop-full`).

You need to install manually if you have installed `ros-melodic-ros-base`.
```
$ sudo apt install ros-melodic-uvc-camera
$ sudo apt install ros-melodic-compressed-image-transport
```

`compressed_image_transport` is required to publish compressed images.
Compressed images are faster to send data through network than raw images.

### Usage
> NOTE: 
> Make sure you run `roscore` in the other session.


The default device is `/dev/video0`. You can run the node with the camera `/dev/video0` as follow:
```
$ rosrun uvc_camera uvc_camera_node
```
You can change the camera device with `_device` option.
```
$ rosrun uvc_camera uvc_camera_node _device:=/dev/video1
```

If you have installed `compressed_image_transport`, `uvc_camera_node` publishes compressed images automatically.

## Image Viewer (Subscriber)
`rqt_image_view` is often used to see images published to a topic.
Just run `rqt_image_view`.
```
$ rqt_image_view
```

## Republish Compressed Images
### Required Package
- [image_transport](http://wiki.ros.org/image_transport)
### Install
```
$ sudo apt install ros-melodic-image-transport
```
### Why Republish?
The type of message between raw image and compressed image is different, and most nodes subscribe `Image` type as a source of image, so you have to **republish** compressed image to **`Image`** type if you utilize compressed image data.
| Topic | Type |
| --- | --- |
| `image_raw` | `sensor_msgs/Image` |
| `image_raw/compressed` | `sensor_msgs/CompressedImage` |

### How to Republish
#### CLI
```
$ rosrun image_transport republish <in_transport> <out_transport> in:=<in_base_topic> out:=<out_base_topic>
```
For example, suppose you are publishing images from a robot using the transport `compressed` and are subscribing `Image` type topic in another node.
Your robot publishes `/image_raw` and `/image_raw/compressed` topics.
The base topic of republished images must be different from `/image_raw`.

```
$ rosrun image_transport republish compressed raw in:=/image_raw out:=/image_repub
```
Note that `<in_base_topic>` must be the base topic(in this exapmle, `/image_raw` is a base topic), not the topic of compressed image topic itself(`/image_raw/compressed`).

#### Launch File
```xml
<arg name="base_camera_image" default="/image_raw"/>
<arg name="image" default="/image_repub" />

<node name="image_republish" pkg="image_transport" type="republish" args="compressed raw">
    <remap from="in" to="$(arg base_camera_image)" />
    <remap from="out" to="$(arg image)" />
</node>
```