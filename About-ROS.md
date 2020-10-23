# About ROS
Robot Operating System(ROS) is a framework being developed by [ROS.org](https://ros.org) for writing robot software.
You can find tutorials and learn more about ROS in [ROS wiki](https://wiki.ros.org).

We think about programs as follows:
- **Sensor A/B**: Read sensor values and send them to **Controller**.
- **Controller**: Calculate motor speed values from the sensor ones, and send them to **Motor**.
- **Motor**: Outputs control signals to motors from the motor speed values.

![](https://i.imgur.com/XC7g8Np.png)

To realize these programs without ROS, it becomes concern how to send/receive data to/from another program.
If you write another program **Controller2** instead of **Controller** and want to swap them, it is necessary to reconnect its data passes.

ROS can help to resolve these problems.

## The Model of ROS
The following figure illustrates the implementation of ROS from **Sensor A/B** to **Controller**.

![](https://i.imgur.com/VEzev8y.png)

The terms in the figure are explained below:

Node
: The program with some functions.

Message
: Nodes send/receive data as *Message*.
Message can include various data structures such as `int32`, `float64`, `string`.

Topic
: *Topic* is like a bulletin board which receives messages.
Each node ***publish***es messages to specific topic, and ***subscribe***s topic to receive its messages.

Thanks to them, each node can publish/subcribe messages without worrying about the other node.

The implementation of ROS from **Controller** to motor operation is illustrated below:

![](https://i.imgur.com/8IJbf90.png)

Node **Controller** subscribes topic `/sensors` and publishes to topic `/motor`.
If you want to swap **Controller** and **Controller2** which subscribes and publishes the same topic, you just run **Controller2** instead of **Controller**. 