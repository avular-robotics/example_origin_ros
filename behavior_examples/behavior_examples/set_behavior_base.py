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

from autonomy_msgs.msg import CoordinateSystemInfo
from geometry_msgs.msg import PoseStamped
from knowledge_base_msgs.msg import Behavior, Constraint, Policy
from knowledge_base_msgs.srv import SetResource, SetAndExecuteBehaviors
from nav_msgs.msg import Odometry
import copy

from scipy.spatial.transform import Rotation as R
import numpy as np
import json
from datetime import datetime


class SetBehaviorBase(Node):

    def __init__(self):
        super().__init__('behavior_set_and_executor')
        # define callback group
        self.mission_executing_callback_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.main_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        # define a local state to store values of the robot's global origin 
        self.coordinate_system_info = {
            "type": 3,
            "local_tangent_plane": {
                "latitude": 0.0,
                "longitude": 0.0,
                "altitude": 0.0,
            }
        }
        # define a local state to store values of the robot's position 
        self.robot_pose = PoseStamped()

        # create a subscription to the actual values of the global origin that the robot is using
        self.coord_info_subscription = self.create_subscription(CoordinateSystemInfo, '/autopilot/coordinate_system', self.coordinate_system_callback, 10)
        self.coord_info_subscription  # prevent unused variable warning
        # create a subscription to the (global) pose of the robot
        self.pose_subscription = self.create_subscription(Odometry, 'autopilot/estimated_pose', self.pose_callback, 10)
        self.pose_subscription  # prevent unused variable warning
              
        # create a service-client for setting a behavior to the robot
        self.add_behavior = self.create_client(
            srv_type=SetAndExecuteBehaviors,
            srv_name='/autopilot/information_manager/set_and_execute_behaviors',
            callback_group=self.main_callback_group,
        )
        # create a service for saving information, such as a path, to the robot
        self.save_path= self.create_client(
            srv_type=SetResource,
            srv_name='/autopilot/information_manager/resource/set',
            callback_group=self.main_callback_group,
        )
        
    
    def coordinate_system_callback(self,msg):
        """
        Function to acquire the coordinate system info and store that on an internal state
        """
        # check if the type of the coordinate system info is not unknown
        if msg.type != 3:
            self.coordinate_system_info["type"] = msg.type
            self.coordinate_system_info["local_tangent_plane"]["latitude"] = msg.latitude
            self.coordinate_system_info["local_tangent_plane"]["longitude"] = msg.longitude
            self.coordinate_system_info["local_tangent_plane"]["altitude"] = msg.altitude
        
    
    def pose_callback(self, msg):
        """
        Function to acquire the robot position and store that on an internal state
        """
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
    
    
    async def create_behavior(self, behavior_type, parameters):
        """
            Function to create the actual list of behaviors that can be sent for execution to the robot
        """
        # create the desired behavior
        # by first checking which behavior_type the user commanded, i.e., a 'move_to_xxx' or a 'follow_path_to_xxx' or a 'wait'
        # once that is checked then define the actual behavior as it is known to the robot and add it to the list of behaviors
        behaviors = []
        if behavior_type == 'move_to_global' or behavior_type == 'move_to_relative':
            waypoint = parameters
            if len(waypoint) == 3:
                if behavior_type == 'move_to_global':
                    behavior = self._create_behavior_move_through_poses(waypoint=parameters, frame="map")
                else:
                    behavior = self._create_behavior_move_through_poses(waypoint=parameters, frame="base_link")
                behaviors.append(behavior)
            else:
                self.get_logger().info('cannot set behavior: unknown number of inputs')
        elif behavior_type == 'follow_path_to_global':
            waypoint = parameters
            if len(waypoint) == 2:
                path = self._compute_straight_path(waypoint=parameters)
                behavior = await self._create_behavior_move_along_path(path=path, coord_system=self.coordinate_system_info)
                behaviors.append(behavior)
            else:
                self.get_logger().info('cannot set behavior: unknown number of inputs')
        elif behavior_type == 'wait':
            waiting_time = parameters[0]
            behavior = Behavior()
            behavior.type = Behavior.WAIT
            behavior.constraint.time = waiting_time
            behaviors.append(behavior)
        else:
            self.get_logger().info('cannot set behavior: unknown type of behavior')
        
        return behaviors
    

    def _create_behavior_move_through_poses(self, waypoint, frame):
        """
            Function to create a move_through_pose behaviors, i.e., where the robot will plan a path by itself
            using the global_costmap (in case of a global waypoint), or the local_costmap (in case of a relative 
            waypoint that is within 3 meters of the robot's current position)
        """
        # create the behavior MOVE_THROUGH_POSES with only one pose being its final goal
        behavior = Behavior()
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
        pose.header.frame_id = frame
        # add some policies on how to execute the behavior
        behavior.goals.append(pose)
        behavior.policy.max_speed = 1.0
        behavior.policy.controller_name = "FollowPathRoughly"           #alternatives are: {FollowPathLoosely, FollowPathRoughly, FollowPathStrictly)

        return behavior


    async def _create_behavior_move_along_path(self, path, coord_system):
        """
            Function to create a move_along_path behavior, i.e., where the robot follows a predefined path
            Note that the path is defined by a list of poses, which in this case are 10 cm apart
            Inter-pose distance may be max 15 cm
        """
        behavior = Behavior()

        # the path should first be saved in the knowledge base of the robot
        # for which we need to create a path dictionary first
        path_dict = {
            "name": 'dummy',
            "creation_date": datetime.now().isoformat(),
            "coordinate_system_info": coord_system,
            "poses": path,
            "constraints": {
                "max_speed_ms": 1.0,
                "min_speed_ms": 0.0,
                "max_height_m": 10.0,
                "min_height_m": 0.05
            }
        }

        # the path dictionary is turned into a resource, which is a predefined JSON format of a path as it is known to the knowledge base
        #   to know the other resource types that are available, please look at /knowledge_base_msgs/msgs/resource_type.msg
        # the resource is then saved to the knowledge base using a service call
        req = SetResource.Request()
        resource_dict = {"type": "path", "resource": path_dict}
        req.json = json.dumps(resource_dict)
        if not self.save_path.service_is_ready():
            response = None
            print("[WARNING]: behavior service not available, aborting...")
        else:
            response = await self.save_path.call_async(req)
    
        # create the MOVE_ALONG_PATH behavior if path is saved successfully, otherwise wait
        if response is None or response.result.success == False:
            behavior.type = Behavior.WAIT
            behavior.constraint.time = 1.0
            print("[WARNING]: failed to save path, sent wait behavior of 1 second instead")
        else:
            print("[INFO]: successfully saved path")
            path_uuid = response.uuid
            behavior.type = Behavior.MOVE_ALONG_PATH
            behavior.constraint.path_uuid = path_uuid
            behavior.policy.max_speed = 1.0
            behavior.policy.max_path_distance = 1.0
            behavior.policy.controller_name = "FollowPathRoughly"           #FollowPathLoosely, FollowPathRoughly, FollowPathStrictly
            
        return behavior


    def _compute_straight_path(self, waypoint):    
        # compute the path as a list of poses, in a straight line, 
        # starting from the robot's current pose and ending at the waypoint
        path = []
        pose = {}
        dx = waypoint[0] - self.robot_pose.pose.position.x
        dy = waypoint[1] - self.robot_pose.pose.position.y
        angle = np.arctan2(dy,dx)
        r = R.from_euler('xyz', [0.0, 0.0, angle], degrees=False)
        quaternions = r.as_quat()
        pose["position"] = {}
        pose["orientation"] = {}
        pose["orientation"]["x"] = quaternions[0]
        pose["orientation"]["y"] = quaternions[1]
        pose["orientation"]["z"] = quaternions[2]
        pose["orientation"]["w"] = quaternions[3]
        # the value '10' in the equation below ensures that a point of the path is created every 0.1 meter, i.e. 10 points in every 1 meter of path
        N = int(np.round( 10 * np.sqrt(float(dx)**2 + float(dy)**2 ) ))
        for i in range(N):
            new_pose = copy.deepcopy(pose)
            new_pose["position"]["x"] = self.robot_pose.pose.position.x + (i/N) * dx
            new_pose["position"]["y"] = self.robot_pose.pose.position.y + (i/N) * dy
            new_pose["position"]["z"] = 0.0
            path.append(new_pose)
        
        return path