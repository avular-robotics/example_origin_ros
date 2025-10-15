# Copyright 2025 Avular Robotics
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

from std_msgs.msg import String
from nav_msgs.msg import Odometry

from scipy.spatial.transform import Rotation as R
import numpy as np


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


def main(args=None):
    rclpy.init(args=args)

    pose2d_subscriber = Pose2DSubscriber()

    rclpy.spin(pose2d_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pose2d_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
