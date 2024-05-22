# Copyright 2023 Avular Robotics
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import numpy as np


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


def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    rclpy.spin(velocity_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
