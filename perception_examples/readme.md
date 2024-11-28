This package defines a ROS2 node to illustrate a perception principles of the Origin. Perception is a critical aspect when creating the world model of a robot, which is the robot's internal representation of the external world. It may hold a virtual fence representing the boundary after which a robot should stop moving further, or it may hold the presence of real-world objects and their position so that the robot can plan a path around them.

In the example that is presented in in this package of the [perception examples](perception_examples/readme.md) we will create a ROS2 node that assesses the LaserScan topic of the robot and uses that to acquire a minimum radius around the robot of free space to maneuver. This minimal radius is computed by checking at what angles (in the horizontal plane) the robot will encounter its nearest obstructions along with the distance to these obstructions. This distance can be interpeded as the minimal free space of the robot.

## get closest obstructions
This example requires the Avular Origin robot <u>and</u> its autonomous capabilities as implemented by Avular's Autopilot Inference (it will thus <u>not</u> work with the Origin One in simulation). In addition you will need a the 3D-LiDAR mounted on the Origin.

To run the example of acquiring the Origin's closest obstruction(s) you need to navigate into the ROS2 workspace and run
```
source install/setup.bash
ros2 run perception_examples get_closest
```
Once the node started it will print a distance and one or more angles in the terminal, each representing the horizontal angle at which the robot will encounter its nearest surrounding objects.

The python code of this ROS2 node is found in the 'perception_examples/perception_examples/get_closest.py'. The remainder of this section explains the code in more detail.

The first part of the code, i.e.,
```
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

import numpy as np
```
imports some typical ROS2 tooling called 'rclpy' and specifically imports the subclass 'Node' which provides us with all functionalities for creating, spinning and killing a ROS2 node. Also, it imports the 'LaseScan' message format of ROS2's geometry messages (geometry_msgs that you have installed with ROS2). You will need to import this format so that the node is able to read the incoming messages properly. The final import is a python package called 'numpy', which you also installed on your local machine. Numpy is a toolbox to compute for Linear Algebra.

The second part of the code, i.e.,
```
class GetClosest(Node):

    def __init__(self):
        super().__init__('laserscan_subscriber')
        # create a subscriber to the robot's laserscan topic
        self.laserscan_subscription = self.create_subscription(LaserScan, 'robot/scan_filtered',  self.getclosest_callback, 10)
        self.laserscan_subscription    # prevent unused variable warning
        self.get_logger().info('laserscan subscriber is initialized')

    def getclosest_callback(self, msg):
        self.get_logger().info('received message')
        # get the laserscan en turn it into a list of distances and angles
        ranges_meter = np.nan_to_num(np.array(msg.ranges), nan = 100)
        # find where the ranges are egual to the range-minimum
        range_min_value = np.min(ranges_meter)
        range_min_element = np.flatnonzero(ranges_meter == ranges_meter.min())
        #compute the angles
        angle_min = msg.angle_min*180/np.pi
        angle_increment = msg.angle_increment*180/np.pi
        angles_where_range_min = []
        for i in range_min_element:
            angles_where_range_min.append(angle_min + i*angle_increment)

        # print the angles at which there is a minimum range to the object
        self.get_logger().info(f"Closest obstruction is at {range_min_value} [cm] and located at angles {angles_where_range_min} [deg]")
```
creates a subclass in which the actual subscriber of the node is defined. This class has an initialization functionality in which the subscriber is created as the internal, or self, object called 'self.laserscan_subscription'. You create a subscriber by defining the message type, in this case 'LaserScan', the topic at which you want to subscribe to, in this case 'robot/scan_filtered', the callback functionality that you want to run once a new message is received, in this case 'self.getclosest_callback', and the number of messages you want to keep in a que to increase robustness of the communication, in this case 10 (although only the last message entering the que will be processed). Once the subscriber is created there is a small statement to avoid any irrelevant warning, followed by a logger which will print to the terminal that the subscriber has been initialized. The LiDAR on the Origin will create a 3D pointcloud. The LaserScan topic that is subscribed to in this ROS2 is a result of the Autopilot Inference after the pointcloud is: 1. filtered so that points reflected from the body of the robot are removed, 2. further filtered by cleaning the pointcloud from outliers and 3. project all points in between 10cm and 70cm above the ground onto a single plane (keep points where 0.1 [m] < z < 0.7 [m] and replace z = 0.2 [m]). 

The second functionality of the class is the callback, which is triggered when a new message is read from the topic 'robot/scan_filtered'. The functionality starts by printing a logging statement to the terminal that a message is received. It then turns the field 'ranges' of this message into a numpy-array, which is then an array of distance measurements to a nearest object at different angles. In case of the Ouster there may be so-called NaN values in this 'ranges' message-field, which occur when the LiDAR could not measure n object within its sensing range. In that case we replace the NaN value with a very high number, in this case 100 [m], to ensure that it will not be the closest object (assuming that there is always an object closer than 100 [m]. The next two lines of code extract the minimal value in this array of ranges and then searches for the element at which this minimal range can be found. The element is a entry-number of the array at which the value at that entry is equal to the minimal value overall. The five lines of code after that are able to compute the angle that corresponds to the entry. It is computed by counting the number of angular increments to a minimal angle of the laserScan, where the number of counts is equal to the entry number found earlier. Finally, the distance and the angles at which this minimal distance was measured is printed as a logging statement to the terminal.

The third part of the code, i.e.,
```
def main(args=None):
    rclpy.init(args=args)

    get_closest_obstructions = GetClosest()

    rclpy.spin(get_closest_obstructions)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_closest_obstructions.destroy_node()
    rclpy.shutdown()
```
creates the main class-functionality, which it to (i) initialize the actual ROS2 node via 'rclpy.init', (ii) define and create the get_closest_obstructions as its class-function via 'get_closest_obstructions = GetClosest()', and further add this subscriber to the ROS2 node for spinning via 'rclpy.spin(get_closest_obstructions)'. The remaining lines of code will ensure that the node is properly destroyed when killed, for example by pressing 'ctrl-c' in the terminal that launched this ROS2 node.

The fourth and final part of the code, i.e.,
```
if __name__ == '__main__':
    main()
```
is a typical python implementation of linking the main functionality of the python routine.