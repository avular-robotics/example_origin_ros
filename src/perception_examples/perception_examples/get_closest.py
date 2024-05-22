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
        super().__init__('velocity_subscriber')
        # create a subscriber to the robot's velocity
        self.laserscan_subscription = self.create_subscription(LaserScan, 'autopilot/laserscan_sliced',  self.getclosest_callback, 10)
        self.laserscan_subscription    # prevent unused variable warning
        self.get_logger().info('velocity subscriber is initialized')

    def getclosest_callback(self, msg):
        # get the laserscan en turn it into a list of distances and angles
        ranges = np.array(msg.ranges)
        ranges = np.int_(10*b)/10
        # find where the ranges are egual to the range-minimum
        range_min_value = np.min(ranges)
        range_min_element = np.argmin(ranges)
        #compute the angles
        angle_min = msg.angle_min*180/np.pi
        angle_increment = msg.angle_increment*180/np.pi
        angles_where_range_min = []
        for i in range_min_element:
            angles_where_range_min.append(angle_min + i*angle_increment)

        # print the angles at which there is a minimum range to the object
        self.get_logger().info(f"Closest obstruction is at {range_min_value} [m] and located at angles {angles_where_range_min} [deg]")


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
