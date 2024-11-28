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

from geometry_msgs.msg import PoseStamped
from knowledge_base_msgs.msg import Behavior, Constraint, Policy
from knowledge_base_msgs.srv import SetAndExecuteBehaviors

#from avular_utils import ClientService
from scipy.spatial.transform import Rotation as R
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
            if future:
                return future.result()
            else:
                return None

class SetBehavior(Node):

    def __init__(self):
        super().__init__('behavior_set_and_executor')
        # create a service-client for setting a behavior to the robot
        self.add_behavior = ClientService(self, srv_type=SetAndExecuteBehaviors, srv_name='/autopilot/information_manager/set_and_execute_behaviors')
        
    def create_behavior(self, behavior_type, parameters):
        # create the desired behavior
        behavior = Behavior()
        behaviors = []
        if behavior_type == 'move_to_global' or behavior_type == 'move_to_relative':
            waypoint = parameters
            if len(waypoint) == 3:
                behavior.type = Behavior.MOVE_THROUGH_POSES
                pose = PoseStamped()
                pose.pose.position.x = float(waypoint[0])
                pose.pose.position.y = float(waypoint[1])
                r = R.from_euler('xyz', [0.0, 0.0, float(waypoint[2])], degrees=True)
                quaternions = r.as_quat()
                pose.pose.orientation.x = quaternions[0]
                pose.pose.orientation.y = quaternions[1]
                pose.pose.orientation.z = quaternions[2]
                pose.pose.orientation.w = quaternions[3]
                if behavior_type == 'move_to_global':
                    pose.header.frame_id = "map"
                else:
                    pose.header.frame_id = "base_link"
                behavior.goals.append(pose)
                behavior.policy.max_speed = 1.0
                behaviors.append(behavior)
            else:
                self.get_logger().info('cannot set behavior: unknown number of inputs')
        elif behavior_type == 'wait':
            waiting_time = parameters[0]
            behavior.type = Behavior.WAIT
            behavior.constraint.time = waiting_time
            behaviors.append(behavior)
        else:
            self.get_logger().info('cannot set behavior: unknown type of behavior')
        
        return behaviors


def main(args=None):
    rclpy.init(args=args)

    set_behavior = SetBehavior()
    
    print("----------------- hello user -----------------------")
    print("define the next behavior")
    print("move_to_global: 1.0 1.0 90.0 ------- [m m deg]")
    print("move_to_relative: 1.0 1.0 0.0 -------- [m m deg]")
    print("wait: 1.0 ---------------------------- [sec]")
    print("quit --------------------------------- stops node \n")
    user_input = input("what is your next command: ")

    while user_input != 'quit':
        n = user_input.find(":")
        behavior_type = user_input[0:n]
        parameter_list = user_input[n+2:].split()
        parameters = []
        for par in parameter_list:
            parameters.append(float(par))
        behaviors = set_behavior.create_behavior(behavior_type, parameters)
        
        if len(behaviors) > 0:
            req = SetAndExecuteBehaviors.Request()
            req.behaviors.extend(behaviors)
            set_behavior.add_behavior.make_request(request_msg = req)
        else:
            print("-------- unknown user input --------")
        user_input = input("what is your next command: ")
        
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    set_behavior.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
