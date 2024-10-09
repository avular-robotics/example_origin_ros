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
from sensor_msgs.msg import LaserScan

import numpy as np


class GetClosest(Node):

    def __init__(self):
        super().__init__('laserscan_subscriber')
        # create a subscriber to the robot's laserscan topic (published by the autopilot inference)
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


def main(args=None):
    rclpy.init(args=args)

    get_closest_obstructions = GetClosest()

    rclpy.spin(get_closest_obstructions)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_closest_obstructions.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()