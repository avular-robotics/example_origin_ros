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

from knowledge_base_msgs.srv import SetAndExecuteBehaviors
from behavior_examples.set_behavior_base import SetBehaviorBase


class SetBehavior(SetBehaviorBase):
    def __init__(self):
        super().__init__()
        
        # define a timer to (continuously) retrieve input from a user
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.request_input,  callback_group = self.mission_executing_callback_group)

        # set an internal state to ensure the callback is not awaiting any previous input
        self.acquiring_input = False

        # print some explanations in the terminal
        print("----------------- hello user -----------------------")
        print("define the next behavior")
        print("move_to_global: 1.0 1.0 90.0 ------- [m m deg]")
        print("move_to_relative: 1.0 1.0 0.0 -------- [m m deg]")
        print("follow_path_to_global: 1.0 1.0 -------- [m m]")
        print("wait: 1.0 ---------------------------- [sec]")
        print("quit --------------------------------- stops node \n")
        
       
    async def request_input(self):

        if not self.acquiring_input and self.coordinate_system_info["type"] == 1:
            self.acquiring_input = True
            await self.get_input()
        else:
            print("[INFO] waiting for the coordinate system of the robot to become available \n")

    
    async def get_input(self):

        user_input = input("what is your next command: ")

        if user_input != 'quit':
            # decompose the input of the user in the type of behavior and its parameters
            n = user_input.find(":")
            behavior_type = user_input[0:n]
            parameter_list = user_input[n+2:].split()
            parameters = []
            for par in parameter_list:
                parameters.append(float(par))    

            # define the behavior based on user input
            behaviors = await self.create_behavior(behavior_type, parameters)

            # sent the behavior to the robot for immediate execution
            if len(behaviors) > 0:
                req = SetAndExecuteBehaviors.Request()
                req.behaviors.extend(behaviors)
                if not self.add_behavior.service_is_ready():
                    print("[WARNING]: behavior service not available, aborting...")
                else:
                    response = await self.add_behavior.call_async(req)
            else:
                print("[INFO] unknown user input")

            # reset the internal state for a next iteration
            self.acquiring_input = False

        else:
            self.destroy_node()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    set_behavior = SetBehavior()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(set_behavior)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        set_behavior.destroy_node()
        rclpy.shutdown()
          
    
if __name__ == '__main__':
    main()
