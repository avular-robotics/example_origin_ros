# Navigation examples
This package defines several ROS2 nodes to illustrate some navigation principles of the Origin. To start, let us briefly present some context of the Origin's navigation functionalities, which are locomotion for moving, localization for estimating its speed and position, and path-planning for navigating between a start and a destination waypoint while avoiding obstacles.

The Avular Origin is a 4-wheeled robot. It moves by following a forward speed and angular rate of its heading. Both may be commanded by the user as is shown in the [third example](#setting-a-reference-speed) of this package. When following speed or angular rate the Origin shall not avoid obstacle, it shall merely try to follow the reference without exploiting its situational awareness.

The Avular Origin uses its IMU and wheel encoder for keeping track of its current current speed and its current position with respect to the real-world position at which the Origin was starte, i.e., the odometry frame. A detailed account on how to acquire the estimated speed of the robot is presented in the [first example](#acquiring-speed). The position in the odometry frame is further transformed into a globally referenced position by intializing the Autopilot Inference when the Origin is placed in front of an Aruco, as the Aruco has a known global position and the Origin is able to localize itself with respect to the Aruco marker. When available, the Origin will further improve its global position estimates with RTK-GNSS measurements and with estimates obtained from matching the LiDAR data to a known map of the environment. An illustration of how to acquire the position estimate of the robot is presented in the [second example](#acquiring-position). Note that in case the Autopilot Inference was not yet completely initialized, for example when no marker has yet been detected, then this estimated position of the Origin will be equal to the odometry position, i.e., its position with respect to the real-wold position wher the Origin was turned on.

The Avular Origin has a path-planning functionality. An example on how to exploit this funcitonality is not presented here in the navigation examples but in the packages on [behavior examples](behavior_examples/readme.md).

## Acquiring speed
This example only requires the Avular Origin robot <u>and not</u> any of its autonomous capabilities as implemented by Avular's Autopilot Inference).

To run the example of acquiring the Origin's position you need to navigate into the 'HelloOrigin' workspace and run
```
source install/setup.bash
ros2 run navigation_examples get_velocity
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
        self.cmdvel_subscription = self.create_subscription(Odometry, 'robot/odom',  self.velocity_callback, 10)
        self.cmdvel_subscription    # prevent unused variable warning
        self.get_logger().info('velocity subscriber is initialized')

    def velocity_callback(self, msg):
        # print the robot's velocity to the terminal
        self.get_logger().info('The robot is driving forward with %.2f [m/s] and turns with %.2f [deg/s]' % (msg.twist.twist.linear.x, msg.twist.twist.angular.z*180/np.pi) )
```
creates a subclass in which the actual subscriber of the node is defined. This class has an intialization functionality in which the subscriber is created as the internal, or self, obect called 'self.cmdvel_subscription'. You create a subscriber by defining the message type, in this case 'Odometry', the topic at which you want to subscribe to, in this case 'robot/odom', the callback functionality that you want to run once a new message is received, in this case 'self.velocity_callback', and the number of messages you want to keep in a que to increase robustness of the communcation, in this case 10 (although only the last message entering the que will be processed). Once the subscriber is created there is a small statement to avoid any irrelevant warning, followed by a logger which will print to the termanal that the subscriber has been intialized. Please note that the topic name of this message start with 'robot', which is to point out that this speed topic is provided by the robot functionality of the Origin, i.e., the functionality that is implements the mechatronic part of the robot and its low-level control software.
The second functionality of the class is the callback, which is triggered when a new message is read from the topic 'robot/odom'. For now, this functionality is merely a logger which will print the forward velocity (being the twist in the linear x direction) and the angular rate of the heading (being the twist rotating around the z-exis) to the terminal. Note that from the odmetry message we are only using the 'twist', which specifies the velocity. In the [second example](acquiring-position) we will unpack the position part of this message.

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
This example only requires the Avular Origin robot <u>and</u> its autonomous capabilities as implemented by Avular's Autopilot Inference. You may create a similar example that does not require the Autopilot Inference by changing the topic of the subscriber from 'autopilot/estimated_pose' to 'robot/odom'.

To run the example of acquiring the Origin's speed you need to navigate into the 'HelloOrigin' workspace and run
```
source install/setup.bash
ros2 run navigation_examples get_2dpose
```
Once the node started it will print the current pose of the robot in the terminal, being its linear position in x and y and its Euler angle around the z-axis (heading). 

The python code of this ROS2 node is found in the 'HelloOrigin/src/navigation_examples/navigation_examples/get_2Dpose.py' and follows the same line of reasoning as for acquiring the velocity with the difference on the frame of this position estimate and how the message is processed in order to retrieve the position and not the velocity. The remainder of this section explains the code in more detail, yet it only explains what is different with respect on the [first example](#acquiring-speed).

The first part of the code, i.e.,
```
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R
import numpy as np
```
imports the same tools as the previous example with the addition of a python package called 'scipy' that you also have installed when setting up these code examples. The 'scipy' package is usefull for goniometric computation, which we will need when transforming quaternion (as used in robotics) into Euler angles (as used for human interpetation). An important note here is that also this topic on which the estimated pose of the robot is being published has the Odometry format. However, I would like to stress that this is merely a simlarity in the format of the message and that the estimated pose as is acquired by the subscription below is defined in a global reference frame and <i>not</i> in the odometry frame.

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
creates a subclass in which the actual subscriber of the node is defined. The initialization is similar to that when acquiring the Origin's speed, with a difference in the topic to which it subscribed, i.e., 'autopilot/estimated_pose', and the callback function that is triggered when a new message is received, i.e., 'pose_callback'. Note that this topic is published by the autopilot functionality, short for Autopilot Inference, of the Origin. The Autopilot Inference is responsible for the Origin's autonomous capabilities, such as autonomous navigation.
The main difference with respect to the previous example of acquiring the speed is in this callback function, which is presented in the second part of this code snippet. Where in the previous example we used the twist part of the incoming message, here we unpack the pose part of the odometry message format. We define 4 variables, 'qx, 'qy', 'qz' and 'qw', which define the orientation of the Origin in the quaternion format. These quaterntion are then used to define a rotation matrix 'r' using the 'scipy' package, and then from 'r' compute the Euler angles. The callback finished with a logger so that the linear position in x and y and the orientation around the z-axis are printed to the terminal. Note that under a typical initialization this pose will be defined in the global reference frame, while if the intialization procedure of the Autopilot Inference in not yet finished, then this topic is either not avialable (yet), or it might be defined in as the pose with respect to the real-world position at which the Origin was starting up.

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
This example only requires the Avular Origin robot <u>and not</u> any of its autonomous capabilities as implemented by Avular's Autopilot Inference).

To run the example of setting the Origin's speed you need to navigate into the 'HelloOrigin' workspace and run
```
source install/setup.bash
ros2 run navigation_examples set_velocity
```
Once the node started it will request the robot to set the control mode of the robot to USER, sent a forward reference speed of 1 m/s for a duration of 1 second, and then return the control model of the robot to a RELEASED state so that the robot can be controlled by other agents or entities, such as the remote control or the Autopilot Inference. 

The python code of this ROS2 node is found in the 'HelloOrigin/src/navigation_examples/navigation_examples/set_velocity.py' in which two service-clients are created and one publisher.

The first part of the code, i.e.,
```
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from origin_msgs.msg import ControlMode
from origin_msgs.srv import SetControlMode
from origin_msgs.srv import ReturnControlMode

import numpy as np

NR_RETRIES = 3
```
imports the similar tools as the previous examples and, in addition, also imports some specific message and service format either specified by open-source packages (geometry_msgs) or dedicated messages for our Origin that are found in this workspace (HelloOrigin/origin_msgs). Also, we define a global parameters NR_RETRIES, which specifies how many times we retry to call a service in case it was not succesfull.

The second part of the code, i.e.,
```
class ClientService():
    
    def __init__(self, node, srv_type, srv_name):
        self.node = node
        self.cli = self.node.create_client(srv_type=srv_type, srv_name=srv_name)

    def make_request(self, request_msg):
        counter = 0
        while not self.cli.wait_for_service(timeout_sec=0.5) and counter < NR_RETRIES:
            self.node.get_logger().info('service not available, waiting again...')
            counter += 1
        if counter == NR_RETRIES:
            self.node.get_logger().info('service not available, abort...')
        else:
            future = self.cli.call_async(request_msg)
            rclpy.spin_until_future_complete(self.node, future)
            self.node.get_logger().info('service call done')               
        return future.result()
```
creates a subclass for a general service-client. It is an extension of service-client one may find in a ROS2 tutorial. The first routine will create the client-service as an internal node. The second routine implements the actual request that the service-client should make. Firstly, it will try to access the server side of the service and check whether it is available. It will to so 3 times (NR_RETRIES) with intervals of half a second. A counter will keep track of how many attempts were made. Then, if all attempts failed, then the server of the service is not available and the service-client is finished. However, it the server is availabe, then the actual call is made to the server. To avoid deadlocks the service call is an asynchronous call and the client-node calling that service will keep on spinnig until the service call is completed. Then, it will notify via the logger that the service call is done and return the result as it is provided by the server back to the client.

The third part of the code, i.e.,
```
lass SetVelocity(Node):

    def __init__(self):
        super().__init__('velocity_commander')
        # create the velocity publisher
        self.velocity_publisher = self.create_publisher(Twist, 'robot/cmd_vel_user', 10)
        self.velocity_msg = Twist()
        self.velocity_msg.linear.x = 0.5 	# meters per second
        self.velocity_msg.angular.z = 0.0	# radians per second
        # create a service for acquiring and releasing control of the robot
        self.request_control = ClientService(self, srv_type=SetControlMode, srv_name='/robot/cmd_vel_controller/set_control_mode')
        self.release_control = ClientService(self, srv_type=ReturnControlMode, srv_name='/robot/cmd_vel_controller/reset_control_mode')
        # create a timer for publising the velocity at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)        
        
    def timer_callback(self):
        # sent velocity commands
        self.velocity_publisher.publish(self.velocity_msg)
        self.get_logger().info('Setting a reference velocity of %.2f [m/s] going forward and %.2f [deg/s] turning ' % (self.velocity_msg.linear.x, self.velocity_msg.angular.z*180/np.pi))
```
creates a second subclass to set the velocity. During intialization one publisher is being created, i.e., 'self.velocity_publisher', that is publishing a velocity to the topic 'robot/cmd_vel_user' that accepts incoming message of the type Twist (which was also used when acquiring the velocity). The 3 lines below this publisher define such a message with a standard velocity of 0.5 m/s forward speed and no angular rate. Then, the next two lines of code create two service-clients that are both of the subclass ClientService, i.e., the general definition of a service-client that we have create directly above (in the second part of the code of this section). A first service-client is for requesting control of the robot by calling a server at '/robot/cmd_vel_controller/set_control_mode'. This server will check what is the current control mode of the robot, which can be NONE, MANUAL, TELE_OPERATED, AUTOPILOT, or USER, and whether the client who is request to change this mode is allowed to MANUAL has the most authority, followed by TELE_OPERATED and finally AUTOPILOT and USER. The second service-client is fo releasing the control mode cby alling a service on "/robot/cmd_vel_controller/reset_control_mode'. This effectively sets the mode to NONE so that a other agent may request control of the robot. The last 2 lines of this intialization creates a timer and a callback whenever the timer hits its sampling rate, in this case 0.1 seconds triggering the 'timer_callback'.
This 'timer_callback' is created in the last part of this code snippet that implements the actual publication of the velocity commands. To do so it requires that the internal variable 'self.velocity_msg' has been set to the desired velocity, as was done during iniatlazation, and then calls the publisher 'self.velocity_publisher' that was created during the intialization to publish the message. Then, a logging statement will be printed in the terminal indicating which velocity reference was published.

The fourth part of the code, i.e.,
```
def main(args=None):
    rclpy.init(args=args)

    set_velocity = SetVelocity()
    
    # request control of the robot's velocity
    req1 = SetControlMode.Request()
    req1.mode.mode = ControlMode.USER
    result = set_velocity.request_control.make_request(request_msg = req1)
    if result.success == True:
        self.get_logger().info('Obtained control of the robot')
        # sent velocity commands
        t_now = set_velocity.get_clock().now().nanoseconds
        t_start = t_now
        t_old = t_now - 100000000               # miliseconds
        while t_now - t_start < 1000000000:     # miliseconds
            t_now = set_velocity.get_clock().now().nanoseconds
            if t_now - t_old > 99999999:
                rclpy.spin_once(set_velocity)
                t_old = t_now
        # request control of the robot's velocity
        req2 = ReturnControlMode.Request()
        req2.mode_from.mode = ControlMode.USER
        result = set_velocity.release_control.make_request(request_msg = req2)
        self.get_logger().info('Done sending velocities, and released control of the robot')
    else:
        self.get_logger().info('Could not obtain control of the robot')
           
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    set_velocity.destroy_node()
    rclpy.shutdown()
```
implements the main strategy for a user to set a reference velocity. It starts by intializing the ROS2 python tools via 'rclpy.init' so that we may create the actual ROS2 node 'set_velocity' that is of the class SetVelocity that we just created here directly above in the third part of the code.
The first thing that must be done before setting a reference velocity is to request control of the robot. This is done by creating a service format of the type that is expected by the service-server, in this case the type is SetControlMode of which we need to define the Request part. Note that we define a USER mode in our Request 'req1'. Then, the call to the service-server is being made by calling the function 'make_request' that is part of the subclass 'request_control' being of the type ClientService, which is further part of the subclass set_velocity that is of the type SetVelocity. Once the request is made a check is made whether control was granted or not, in which case a logging statement is printed to the terminal whether or not control was obtained (if-then-else)
In case it was granted that some time instances are defined with a current time, a starting time and an end time during which the while-loop will be active. The while loop starts by obtaining the current time as it is recorded by our main node 'set_velocity. We are using this time instead of the computer time to avoid errors when running a simulation with this ROS2 node that is running faster or slower than real-time. The other part of the while-loop checks if already 0.1 seconds, or 999999 miliseconds, were passed, and if that is the case it will spin the ROS2 node once. During this spin the 'timer_callback' of 'set_velocity' will be triggered and a new velocity reference will be published.
Once the while-loop is finished, after 1 second, we use a similar set of lines for releasing the control of the robot, from USER mode, as we did when requesting control of the robot. A confirmation is printed to the terminal by the logger indicating that all desired velocities were sent and control of the robot has been released.
The last few lines of this code snippet are similar to the other examples for destrying the ROS2 node properly in case it is being killed.

The fifth and final part of the code, i.e.,
```
if __name__ == '__main__':
    main()
```
is a typical python implementation of linking the main functionality of the python routine.
