# Navigation examples
This package defines several ROS2 nodes to illustrate some navigation principles of the Origin. To start, let us briefly present some context of the Origin's navigation functionalities, which are locomotion for moving, localization for estimating its speed and position, and path-planning for navigating between a start and a destination waypoint.

The Avular Origin is a 4-wheeled robot. It moves by following a forward speed and angular rate of its heading. Both may be commanded by the user as is shown in the [third example](#setting-a-reference-speed) of this package. When following speed or angular rate the Origin shall not avoid obstacle, it shall merely try to follow the reference without exploiting its situational awareness.

The Avular Origin uses its IMU and wheel encoder for keeping track of its current current speed and its current position with respect to its starting position (odometry). An illustration of how to acquire the estimated speed of the robot is presented in the [first example](#acquiring-speed). The position in the odometry frame is further transformed into a globally referenced position by intializing the Autopilot Inference when the Origin is placed in front of an Aruco, as the Aruco has a known global position and the Origin is able to localize itself with respect to the Aruco marker. When available, the Origin will further improve its global position estimates with RTK-GNSS measurements and with estimates obtained from matching the LiDAR data to a known map of the environment. An illustration of how to acquire the position estimate of the robot is presented in the [second example](#acquiring-position). Note that in case the Autopilot Inference was not yet completely initialized, for example when no marker has yet been detected, then this estimated position of the Origin will be equal to the odometry position, i.e., its position with respect to the real-wold position wher the Origin was turned on.

The Avular Origin has a path-planning functionality. An example on how to exploit this funcitonality is not presented here in the navigation examples but in the packages on [behavior examples](behavior_examples/readme.md).

## Acquiring speed
To run the example of acquiring the Origin's position you need to navigate into the 'HelloOrigin' workspace and run
```
source install/setup.bash
ros2 run navigation_examples get_velocity.py
```
Once the node started it will print the current speed and angular rate in the terminal. 

The python code of this ROS2 node is found in the 'HelloOrigin/src/navigation_examples/navigation_examples/get_velocity.py'. The remainder of this section explains the code in more detail.

The first part of the code, i.e.,
```
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import numpy as np
```
imports some typical ROS2 tooling called 'rclpy' and specifically imports the subclass 'Node' which provides us with all functionalities for creating, spinning and killing a ROS2 node. Also, it imports the 'Odometry' message format of the NAV2 stack (nav_msgs that you have installed in addition to this workspace). You will need to import this format so that the node is able to read the incoming messages properly. The final import is a python package called 'numpy', which you also installed on your local machine. Numpy is a toolbox to compute for Linear Algebra.

The second part of the code, i.e.,
```
class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')
        # create a subscriber to the robot's velocity
        self.cmdvel_subscription = self.create_subscription(Odometry, 'autopilot/estimated_pose',  self.velocity_callback, 10)
        self.cmdvel_subscription    # prevent unused variable warning
        self.get_logger().info('velocity subscriber is initialized')

    def velocity_callback(self, msg):
        # print the robot's velocity to the terminal
        self.get_logger().info('The robot is driving forward with %.2f [m/s] and turns with %.2f [deg/s]' % (msg.twist.twist.linear.x, msg.twist.twist.angular.z*180/np.pi) )
```
creates a subclass in which the actual subscriber of the node is defined. This class has an intialization functionality in which the subscriber is created as the internal, or self, obect called 'self.cmdvel_cubscription'. You create a subscriber by defining the message type, in this case 'Odometry', the topic at which you want to subscribe to, in this case 'autopilot/estimated_pose', the callback functionality that you want to run once a new message is received, in this case 'self.velocity_callback', and the number of messages you want to keep in a que to increase robustness of the communcation, in this case 10 (although only the last message entering the que will be processed). Once the subscriber is created there is a small statement to avoid any irrelevant warning, followed by a logger which will print to the termanal that the subscriber has been intialized.
The second functionality of the class is the callback, which is triggered when a new message is read from the topic 'autopilot/estimated_pose'. For now, this functionality is merely a logger which will print the forward velocity (being the twist in the linear x direction) and the angular rate of the heading (being the twist rotating around the z-exis) to the terminal. Note that from the odmetry message we are only using the 'twist', which specifies the velocity. In the [second example](acquiring-position) we will unpack the position part of this message.

The third part of the code, i.e.,
```
def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    rclpy.spin(velocity_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_subscriber.destroy_node()
    rclpy.shutdown()
```
creates the main class-functionality, which it to (i) intialize the actual ROS2 node via 'rclpy.init', (ii) define and create the velocity_subscriber as its class-function via 'velocity_subscriber = VelocitySubscriber()', and further add this subscriber to the ROS2 node for spinning via 'rclpy.spin(velocity_subscriber)'. The remainig lines of code will ensure that the node is properly destroyed when killed, for example by pressing 'ctrl-c' in the terminal that launched this ROS2 node.

The fourth and final part of the code, i.e.,
```
if __name__ == '__main__':
    main()
```
is a typical python implementation of linking the main functionality of the python routine.

## Acquiring position
To run the example of acquiring the Origin's speed you need to navigate into the 'HelloOrigin' workspace and run
```
source install/setup.bash
ros2 run navigation_examples get_2dpose.py
```
Once the node started it will print the current pose of the robot in the terminal, being its linear position in x and y and its Euler angle around the z-axis (heading). 

The python code of this ROS2 node is found in the 'HelloOrigin/src/navigation_examples/navigation_examples/get_2Dpose.py' and follows the same line of reasoning as for acquiring the velocity with the difference on the actual substription and how the message is processed. The remainder of this section explains the code in more detail, yet it only explains what is different with respect on the [first example](#acquiring-speed).

The first part of the code, i.e.,
```
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R
import numpy as np
```
imports the same tools as the previous example with the addition of a python package called 'scipy' that you also have installed when setting up these code examples. The 'scipy' package is usefull for goniometric computation, which we will need when transforming quaternion (as used in robotics) into Euler angles (as used for human interpetation).

The second part of the code, i.e.,
```
class Pose2DSubscriber(Node):

    def __init__(self):
        super().__init__('pose2d_subscriber')
        self.pose_subscription = self.create_subscription(Odometry, 'autopilot/estimated_pose', self.pose_callback, 10)
        self.pose_subscription  # prevent unused variable warning
        self.get_logger().info('pose subscriber is initialized')

    def pose_callback(self, msg):
        self.robot_pose = msg
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        r = R.from_quat([qx, qy, qz, qw])
        euler_rotation = r.as_euler('xyz', degrees=True)
        self.get_logger().info('The robot is at X = %.2f [m], Y = %.2f [m], and has a heading of %.2f' % (msg.pose.pose.position.x, msg.pose.pose.position.y, euler_rotation[2]) )
```
creates a subclass in which the actual subscriber of the node is defined. The initialization is similar to that when acquiring the Origin's speed, with the difference in the topic that is being subscribed to 'autopilot/estimated_pose' and the callback function that is triggered when a new message is received.
The main different with respect to the previous example of acquiring the speed is in the callback function, i.e., 'pose_callback'. Where in the previous example we used the twist part of the message, here we unpack the pose part of the odometry message format. We define 4 variables, 'qx, 'qy', 'qz' and 'qw', which define the orientation of the Origin in the quaternion format. These quaterntion are then used to define a rotation matrix 'r' using the 'scipy' package, and then from 'r' compute the Euler angles. The callback finished with a logger so that the linear position in x and y and the orientation around the z-axis are printed to the terminal. Note that under a typical initialization this pose will be defined in the global reference frame, while if the intialization procedure of the Autopilot Inference in not yet finished, then this topic is either not avialable (yet), or it might be defined in as the pose with respect to the real-world position at which the Origin was starting up.

The third part of the code, i.e.,
```
def main(args=None):
    rclpy.init(args=args)

    pose2d_subscriber = Pose2DSubscriber()

    rclpy.spin(pose2d_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose2d_subscriber.destroy_node()
    rclpy.shutdown()
```
creates the main class-functionality that is similar to the previous example to acquire the speed yet creates the 2D pose subscriber instead of a velocity subscriber.

The fourth and final part of the code, i.e.,
```
if __name__ == '__main__':
    main()
```
is a typical python implementation of linking the main functionality of the python routine.


## Setting a reference speed