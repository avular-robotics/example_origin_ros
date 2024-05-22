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

from geometry_msgs.msg import Twist
from origin_msgs.msg import ControlMode
from origin_msgs.srv import SetControlMode
from origin_msgs.srv import ReturnControlMode

#from avular_utils import ClientService
import numpy as np
import time

NR_RETRIES = 3

class ClientService():
    def __init__(self, node, srv_type, srv_name):
        self.node = node
        self.cli = node.create_client(srv_type=srv_type, srv_name=srv_name)

    def make_request(self, request_msg):
        text = 'service call was un-succesfull'
        counter = 0
        while not self.cli.wait_for_service(timeout_sec=0.5) and counter < NR_RETRIES:
            self.node.get_logger().info('service not available, waiting again...')
            counter += 1
        if counter == NR_RETRIES:
            self.node.get_logger().info('service not available, abort...')
        else:
            self.node.future = self.cli.call_async(request_msg)
            self.node.get_logger().info('service call done')
            print(self.node.future.result())
            text = 'success'                
        return text

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_commander')
        # create the velocity publisher
        self.velocity_publisher_ = self.create_publisher(Twist, 'robot/cmd_vel', 10)
        # create a service for acquiring and releasing control of the robot
        self.sub_node_1 = rclpy.create_node('request_control')
        self.request_control = self.sub_node_1.create_client(srv_type=SetControlMode, srv_name='cmd_vel_controller/set_control_mode')
        self.sub_node_2 = rclpy.create_node('release_control')
        self.release_control = self.sub_node_2.create_client(srv_type=ReturnControlMode, srv_name='cmd_vel_controller/reset_control_mode')
        # create a timer for periodically sending velocities       
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.get_logger().info('velocity commander is initialized')
    
    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5 	# meters per second
        msg.angular.z = 0.0	# radians per second
        if self.i == 0:
        	# request control of the robot's velocity
            req1 = SetControlMode.Request()
            req1.mode.mode = ControlMode.USER
            self.request_control.wait_for_service()
            future = self.request_control.call_async(req1)
            rclpy.spin_until_future_complete(self.sub_node_1, future)
            print(future.result())
        elif self.i < 10:
        	# sent velocity commands
        	self.velocity_publisher_.publish(msg)
        	self.get_logger().info('Setting a reference velocity of %.2f [m/s] going forward and %.2f [deg/s] turning ' % (msg.linear.x, msg.angular.z*180/np.pi))
        elif self.i < 11:
            # stop velcity commands and release the robot's control
            req2 = ReturnControlMode.Request()
            req2.mode_from.mode = ControlMode.USER
            self.release_control.wait_for_service()
            future = self.release_control.call_async(req2)
            rclpy.spin_until_future_complete(self.sub_node_2, future)
            print(future.result())
            self.get_logger().info('velocity has been sent for 1 second')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    velocity_publisher = VelocityPublisher()

    rclpy.spin(velocity_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    velocity_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
