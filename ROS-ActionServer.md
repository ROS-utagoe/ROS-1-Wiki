# ROS Action Server and Client Model

### Required Package
- [actionlib](http://wiki.ros.org/actionlib)

### Refereces
- [actionlib_tutorials](http://wiki.ros.org/actionlib_tutorials/Tutorials)
- [ROS講座95 actionlibを使う](https://qiita.com/srs/items/a39dcd24aaeb03216026)

---

# Creating the Action Message
## For Message Package `yolo_object_msgs`
#### `/action/ApproachToObject.action`
```
# Define the goal
string class_name
---
# Define the result
geometry_msgs/Pose final_pose
---
# Define a feedback message
geometry_msgs/Pose current_pose
```

#### `CMakeLists.txt`
```
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  actionlib_msgs
)

...

add_action_files(
  DIRECTORY action
  FILES
  ApproachToObject.action
)

...

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)
```

#### `package.xml`
```xml
<buildtool_depend>catkin</buildtool_depend>
<build_depend>geometry_msgs</build_depend>
<build_depend>actionlib_msgs</build_depend>
<build_depend>actionlib</build_depend>
<build_depend>std_msgs</build_depend>
<build_export_depend>geometry_msgs</build_export_depend>
<build_export_depend>std_msgs</build_export_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>actionlib_msgs</exec_depend>
<exec_depend>actionlib</exec_depend>
<exec_depend>std_msgs</exec_depend>
<exec_depend>message_generation</exec_depend>
```

## catkin_make
After running `catkin_make`, you'll see these messages generated in `~/catkin_ws/devel/share/yolo_object_msgs/msg/`:
- ApproachToObjectAction.msg
- ApproachToObjectActionGoal.msg
- ApproachToObjectActionResult.msg
- ApproachToObjectActionFeedback.msg
- ApproachToObjectGoal.msg
- ApproachToObjectResult.msg
- ApproachToObjectFeedback.msg

## For Dependent Packages
#### `CMakeLists.txt`
```
find_package(catkin REQUIRED COMPONENTS
  ...
  yolo_object_msgs
)
generate_messages(
  DEPENDENCIES
  yolo_object_msgs
)
catkin_package(
  CATKIN_DEPENDS yolo_object_msgs
)
```

# Usage
### Action Server Example (Python)
```python
#! /usr/bin/env python

import rospy
import actionlib

from yolo_object_msgs.msg import ApproachToObjectAction, ApproachToObjectActionResult, ApproachToObjectActionFeedback


class ApproachToObjectServer:
	def __init__(self):
		self.action_server = actionlib.SimpleActionServer('object_approacher', ApproachToObjectAction, self.execute, False)
		self.action_server.start()

	def execute(self, goal):
		# Do lots of awesome groundbreaking robot stuff here
		self.action_server.set_succeeded()


if __name__ == '__main__':
	rospy.init_node('do_dishes_server')
	server = ApproachToObjectServer()
	rospy.spin()

```

### Action Client Example (Python)
```python
#!/usr/bin/env python
import rospy
import actionlib
from yolo_object_msgs.msg import ApproachToObjectAction, ApproachToObjectGoal


def approach_to_object():
    action_client = actionlib.SimpleActionClient('object_approacher', ApproachToObjectAction)
    action_client.wait_for_server()

    goal = ApproachToObjectGoal()
    goal.class_name = 'cup'
    action_client.send_goal(goal)
    action_client.wait_for_result()


if __name__ == "__main__":
    try:
        rospy.init_node('object_approacher_client')
        approach_to_object()
    except rospy.ROSInterruptException:
        pass

```
