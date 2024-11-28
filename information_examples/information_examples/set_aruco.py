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

from knowledge_base_msgs.msg import Behavior, ResourceType, ModesOfOperation
from knowledge_base_msgs.srv import ListResource, GetResource, SetResource, DeleteResource, SaveMap, SetModeOfOperation
from autonomy_msgs.msg import CoordinateSystemInfo

import json
import uuid
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Rotation as R
import numpy as np

class ClientService():
    
    def __init__(self, node, srv_type, srv_name):
        self.node = node
        self.cli = self.node.create_client(srv_type=srv_type, srv_name=srv_name)

    def send_request(self, request_msg):
        is_called = False
        if not self.cli.wait_for_service(timeout_sec=0.5):
            self.node.get_logger().debug("service not available, aborting...")
            return is_called, None

        future = self.cli.call_async(request_msg)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            return is_called, None
        is_called = True
        
        return is_called, future.result()


class SetAruco(Node):
    def __init__(self):
        super().__init__('aruco_setter')
        self.coordinate_system_info = {
            "type": 3,
            "local_tangent_plane": {
                "latitude": 0.0,
                "longitude": 0.0,
                "altitude": 0.0,
            }
        }
        # create a service-client for setting a the id and pose of an aruco marker
        self.list_information = ClientService(self, srv_type=ListResource, srv_name='/autopilot/information_manager/resource/list')
        self.get_information = ClientService(self, srv_type=GetResource, srv_name='/autopilot/information_manager/resource/get')
        self.set_information = ClientService(self, srv_type=SetResource, srv_name='/autopilot/information_manager/resource/set')
        self.delete_information = ClientService(self, srv_type=DeleteResource, srv_name='/autopilot/information_manager/resource/delete')
        # create subscribers
        self.subscription_coord_info = self.create_subscription(CoordinateSystemInfo, '/autopilot/coordinate_system', self.coordinate_system_callback, 10)

    def coordinate_system_callback(self,msg):
        """
        Function to absorb the coordinate system info and put that on an internal state
        """
        # check if the type of the coordinate system info is not unknown
        if msg.type != 3:
            self.coordinate_system_info["type"] = msg.type
            self.coordinate_system_info["local_tangent_plane"]["latitude"] = msg.latitude
            self.coordinate_system_info["local_tangent_plane"]["longitude"] = msg.longitude
            self.coordinate_system_info["local_tangent_plane"]["altitude"] = msg.altitude
    
    def create_marker_dict(self, marker_id, marker_pose):
        """Function to check if the marker already exists in the database, and thus we should delete it"""
        is_existing = False
        marker_dict = {}
        if marker_id >= 0:
            # list all known markers from the knowledge base
            req = ListResource.Request()
            resource_type = ResourceType()
            resource_type.type = ResourceType.MARKER
            req.type = resource_type
            is_called, response = self.list_information.send_request(req)
            marker_list = []
            if is_called:
                marker_list = json.loads(response.json)
            # acquire the information from the knowledge base per marker and memorize that information of it matches our marker_id  
            for item in marker_list:
                req = GetResource.Request()
                req.uuid = str(item['resource']['id'])
                req.type.type = ResourceType.MARKER
                is_called, response = self.get_information.send_request(req)
                if is_called:
                    if response.result.success:
                        marker_received = json.loads(response.json)
                        if marker_id == int(marker_received['resource']['marker_id']):
                            marker_dict = marker_received['resource']
                            marker_dict["coordinate_system_info"] = self.coordinate_system_info
                            marker_dict['pose'] = marker_pose
                            is_existing = True
            if not is_existing:
                marker_dict = {
                    "id": str(uuid.uuid4()),
                    "group_id": str(uuid.uuid4()),
                    "name": f"aruco_marker_{marker_id}",
                    "marker_id": marker_id,
                    "coordinate_system_info": self.coordinate_system_info,
                    "pose": marker_pose,
            }
        else:
            self.get_logger().info('provided an incorrect marker id')

        return is_existing, marker_dict


def main(args=None):
    rclpy.init(args=args)

    set_aruco = SetAruco()

    while set_aruco.coordinate_system_info['type'] == 3:
        print('waiting to receive coordinate system info that is not of the type UNKNOWN ... press ctrl-c to kill waiting (and node)')
        rclpy.spin_once(set_aruco)
        print('received new ROS2 messages ... processing')
    
    print('\n')
    print("------------------------- hello user --------------------------")
    print("Define a new or updated pose of an Aruco marker")
    print("marker_id: X_global, Y_global, Heading_global ------- [m m deg]")
    print("For example, to add a marker_id you insert the following,")
    print("3: 21.0 13.0 90.0")
    print("quit --------------------------------- stops node \n")
    user_input = input("what is your next command: ")

    while user_input[0:4] != 'quit':
        feedback = "------- saving was unsuccessful ------------"
        n = user_input.find(":")
        marker_id = int(user_input[0:n])
        parameter_list = user_input[n+2:].split()
        if len(parameter_list) == 3:
            rot_matrix = R.from_euler('zyx', [float(parameter_list[2]), 0, 90.0], degrees=True)
            quaternion = rot_matrix.as_quat()
            marker_pose = { 
                'position': {
                    'x': float(parameter_list[0]), 
                    'y': float(parameter_list[1]),
                    'z': 0.3,
                },
                'orientation': {
                    'x': quaternion[0],
                    'y': quaternion[1],
                    'z': quaternion[2],
                    'w': quaternion[3]
                }
            }
            [is_existing, marker_dict] = set_aruco.create_marker_dict(marker_id, marker_pose)
            is_deleted = False
            if is_existing:
                # delete the old marker if it already exists
                req = DeleteResource.Request()
                resource_type = ResourceType()
                resource_type.type = ResourceType.MARKER
                req.uuid = str(marker_dict["id"])
                req.type = resource_type
                is_deleted, response = set_aruco.delete_information.send_request(req)
            if ( not is_existing or (is_existing and is_deleted) ) and len(marker_dict) > 0:
                req = SetResource.Request()
                resource_dict = {"type": "marker", "resource": marker_dict}
                req.json = json.dumps(resource_dict)
                is_called, response = set_aruco.set_information.send_request(req)
                if is_called:
                    if response.result.success:
                        feedback = "------- successfully saved marker ------------"
                    else:
                        feedback = f"------- not saved marker: {response.result.message} ---------"
            else:
                feedback = "--------- not all marker info could be acquired ----------"
            print(feedback)
        else:
            print("-------- unknown user input --------")
        
        user_input = input("what is your next command: ")
        
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    set_aruco.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
