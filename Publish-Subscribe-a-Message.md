###### tags: `ROS` `Raspberry Pi`

Publish/Subscribe a Message
===

# Message Setup
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

## 2.Edit package.xml
Make sure these two lines are uncommented:
```
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

## 3.Edit CMakeLists.txt
You have to edit `CMakeLists.txt` file to activate your `*.msg` file.

Make sure you add `std_msgs` and `message_generation` to the list of COMPONENTS and export `message_runtime` dependency in `catkin_package`:
```
find_package(catkin REQUIRED COMPONENTS
   roscpp
   rospy
   std_msgs
   message_generation
)

...

catkin_package(
    ...
    CATKIN_DEPENDS ... message_runtime ...
    ...
)
```
Next, edit `add_message_files` and `generate_messages` blocks like this:
```
add_message_files(
    FILES
    Motor.msg
)

...

generate_messages(
  DEPENDENCIES
  std_msgs
)
```

Finally, you have to run `catkin_make`

    $ cd ~/catkin_ws
    $ catkin_make

# Publisher/Subscriber Node (Python)
Let's create publisher/subscriber nodes with Python.
In this section, we implement this:

![](https://i.imgur.com/8IJbf90.png)


In this example, node **Controller** is a ***Publisher*** and node **Motor** is a ***Subscriber***.
Node **Controller** doesn't subscribe any topic here.

## Publiser (Controller.py)
```=python
#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
from <Package Name>.msg import Motor

def Controller():

    rospy.init_node('controller', anonymous=True, disable_signals=True)
    # Publish message to Topic named '/motor'
    pub = rospy.Publisher('motor', Motor, queue_size=10)

    msg = Motor()

    msg.direction = 0
    msg.speed = 0.5

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(msg)
        r.sleep()

if __name__ == "__main__":
    try:
        Controller()
    except rospy.ROSInterruptException:
        pass
```



## Subscriber (Motor.py)
