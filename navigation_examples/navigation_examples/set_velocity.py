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

import numpy as np

NR_RETRIES = 3

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
            if hasattr(future, 'result'):
                return future.result()
            else:
                return None

class SetVelocity(Node):

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
        # create a timer for publishing the velocity at 10 Hz
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)        
        
    def timer_callback(self):
        # sent velocity commands
        self.velocity_publisher.publish(self.velocity_msg)
        self.get_logger().info('Setting a reference velocity of %.2f [m/s] going forward and %.2f [deg/s] turning ' % (self.velocity_msg.linear.x, self.velocity_msg.angular.z*180/np.pi))


def main(args=None):
    rclpy.init(args=args)

    set_velocity = SetVelocity()
    
    # request control of the robot's velocity
    req1 = SetControlMode.Request()
    req1.mode.mode = ControlMode.USER
    result = set_velocity.request_control.make_request(request_msg = req1)
    if result:
        success = result.success
    else:
        success = False
    if success:
        set_velocity.get_logger().info('Obtained control of the robot')
        # sent velocity commands
        t_now = set_velocity.get_clock().now().nanoseconds
        t_start = t_now
        t_old = t_now - 100000000               # milliseconds
        while t_now - t_start < 1000000000:     # milliseconds
            t_now = set_velocity.get_clock().now().nanoseconds
            if t_now - t_old > 99999999:
                rclpy.spin_once(set_velocity)
                t_old = t_now
        # request control of the robot's velocity
        req2 = ReturnControlMode.Request()
        req2.mode_from.mode = ControlMode.USER
        result = set_velocity.release_control.make_request(request_msg = req2)
        set_velocity.get_logger().info('Done sending velocities, and released control of the robot')
    else:
        set_velocity.get_logger().info('Could not obtain control of the robot')
           
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    set_velocity.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
