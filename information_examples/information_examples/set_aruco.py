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

from knowledge_base_msgs.msg import Behavior, ResourceType, ModesOfOperation
from knowledge_base_msgs.srv import ListResource, GetResource, SetResource, DeleteResource, SetRobotCoordInfo
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

        self.node.get_logger().debug("making request...")
        future = self.cli.call_async(request_msg)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is None:
            return is_called, None
        is_called = True
        
        return is_called, future.result()


class SetLtpAndAruco(Node):
    def __init__(self):
        super().__init__('local_tangent_plane_and_aruco_setter')
        self.coordinate_system_info = {
            "type": 3,
            "local_tangent_plane": {
                "latitude": 0.0,
                "longitude": 0.0,
                "altitude": 0.0,
            }
        }
        self.user_defined_coord_system = False
        # create a service-client for setting a the id and pose of an aruco marker
        self.list_information = ClientService(self, srv_type=ListResource, srv_name='/autopilot/information_manager/resource/list')
        self.get_information = ClientService(self, srv_type=GetResource, srv_name='/autopilot/information_manager/resource/get')
        self.set_information = ClientService(self, srv_type=SetResource, srv_name='/autopilot/information_manager/resource/set')
        self.delete_information = ClientService(self, srv_type=DeleteResource, srv_name='/autopilot/information_manager/resource/delete')
        self.set_coord_system = ClientService(self, srv_type=SetRobotCoordInfo, srv_name='/autopilot/information_manager/set_stored_robot_coord_info')
        # create subscribers
        self.subscription_coord_info = self.create_subscription(CoordinateSystemInfo, '/autopilot/coordinate_system', self.coordinate_system_callback, 10)

    def coordinate_system_callback(self,msg):
        """
        Function to acquire the coordinate system info and put that on an internal state
        """
        # check if the type of the coordinate system info is not unknown
        if msg.type != 3 and not self.user_defined_coord_system:
            self.coordinate_system_info["type"] = msg.type
            self.coordinate_system_info["local_tangent_plane"]["latitude"] = msg.latitude
            self.coordinate_system_info["local_tangent_plane"]["longitude"] = msg.longitude
            self.coordinate_system_info["local_tangent_plane"]["altitude"] = msg.altitude
    
    def print_all_markers(self):
        """Function to acquire all markers that already exists in the database"""
        # list all known markers from the knowledge base
        req = ListResource.Request()
        resource_type = ResourceType()
        resource_type.type = ResourceType.MARKER
        req.type = resource_type
        is_called, response = self.list_information.send_request(req)
        marker_list = []
        if is_called:
            marker_list = json.loads(response.json)
        else:
            print("\n--> Could not find any marker in the database, or did not receive a response from the database")
            return
        
        # acquire the information from the knowledge base per marker and memorize that information of it matches our marker_id  
        for item in marker_list:
            req = GetResource.Request()
            req.uuid = str(item['resource']['id'])
            req.type.type = ResourceType.MARKER
            is_called, response = self.get_information.send_request(req)
            if is_called:
                if response.result.success:
                    marker_received = json.loads(response.json)
                    rot_matrix = R.from_quat([marker_received['resource']['pose']['orientation']['x'], marker_received['resource']['pose']['orientation']['y'], marker_received['resource']['pose']['orientation']['z'], marker_received['resource']['pose']['orientation']['w']])
                    euler = rot_matrix.as_euler('xyz', degrees=True)
                    x_pos = marker_received['resource']['pose']['position']['x']
                    y_pos = marker_received['resource']['pose']['position']['y']
                    heading = euler[2]
                    print(f"--> Found marker with ID {marker_received['resource']['marker_id']}, "
                            f"located at global position x = {x_pos}, y = {y_pos} and heading = {heading}, "
                            f"from the coordinate system {json.dumps(marker_received['resource']['coordinate_system_info']['local_tangent_plane'])} ")
                else:
                    print("\n--> Could not obtain the information for one specific marker")
            else:
                print("\n--> Could not obtain the information for one specific marker")
        return
        
    
    
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
                # combine all the information of a marker in a dictionary
                # these element of the dictionary are required before the knowledge base of the robot is able to save the marker
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

    set_ltp_aruco = SetLtpAndAruco()

    print("\n-----------------------------------------------------------------------")
    print("----------------------------- hello user ------------------------------")
    print("-----------------------------------------------------------------------")
    
    # at first check if the user wants to set a new coordinate system for the robot
    correct_input = False
    while not correct_input:
        user_input = input("Do you want to update the values of the coordinate system in the database of the robot? [y/n]: ")

        # in case the user wants to set a new coordinate system, then acquire those values
        if user_input[0] == 'y':
            correct_input = True
            correct_nr_parameters = False
            while not correct_nr_parameters:
                print("\nPlease provide the desired values for latitude, longitude and altitude.")
                print("For example, to set latitude = 51.453565613608696, longitude = 5.448594918812281 and altitude = 64.14> simply type")
                print("51.4535656 5.4485949 64.14")
                user_input = input("\nWhat values would you like to set: ")
                parameter_list = user_input.split()
                
                # store the new lat lon and alt into the robot's database
                if user_input[0:4] == 'quit':
                    correct_nr_parameters = True
                elif len(parameter_list) == 3:
                    correct_nr_parameters = True
                    req = SetRobotCoordInfo.Request()
                    req.coord_info.type = 1
                    req.coord_info.latitude = float(parameter_list[0])
                    req.coord_info.longitude = float(parameter_list[1])
                    req.coord_info.altitude = float(parameter_list[2])
                    is_called, response = set_ltp_aruco.set_coord_system.send_request(req)
                    # also store the new coordinate system in the local state "coordinate_system" for later usage
                    if is_called:
                        if response.success:
                            feedback = "\n--> Successfully saved coordinate system"
                            set_ltp_aruco.user_defined_coord_system = True
                            set_ltp_aruco.coordinate_system_info["type"] = 1
                            set_ltp_aruco.coordinate_system_info["local_tangent_plane"]["latitude"] = float(parameter_list[0])
                            set_ltp_aruco.coordinate_system_info["local_tangent_plane"]["longitude"] = float(parameter_list[1])
                            set_ltp_aruco.coordinate_system_info["local_tangent_plane"]["altitude"] = float(parameter_list[2])
                        else:
                            feedback = f"--> Could not save coordinate system: {response.message}"
                    else:
                        feedback = f"--> Could not saved coordinate system: {response.message}"
                    print(feedback)
                else:
                    print('\nThe input was not of the right format. Please try again, or type "quit"')
        
        # if the user does not want to set a new coordinate system, then wait until the robot has published its coordinate system
        elif user_input[0] == 'n':
            correct_input = True
            while set_ltp_aruco.coordinate_system_info['type'] == 3:
                print('\n--> Waiting to receive coordinate system info that is not of the type UNKNOWN ... press ctrl-c to kill waiting (and node)')
                rclpy.spin_once(set_ltp_aruco)
                print('\r--> Received new ROS2 messages ... processing')
       
        # if the input is not one of the above 2 option, then try again
        else:
            print("unknown input")
    
    print("\n-----------------------------------------------------------------------")
    print("-------------------- acquire current Aruco markers --------------------")
    print("-----------------------------------------------------------------------")
    set_ltp_aruco.print_all_markers()
    
    # next, continue to storing aruco markers
    print("\n------------------------------------------------------------------------")
    print("------------------------- store aruco markers --------------------------")
    print("------------------------------------------------------------------------")
    print("Define a new or updated pose of an Aruco marker in the format <marker_id: X_global, Y_global, Heading_global> in [m m deg]")
    print("For example, to add an Aruco marker with ID=3 you may type: 3: 21.0 13.0 90.0")
    print("In case you want to quit this example node, please type: quit")
    user_input = input("\nWhat new Aruco marker would you like to set (or type 'quit' to stop): ")

    # continue to request a next marker as long as the user did not wanted to quit
    while user_input[0:4] != 'quit':
        feedback = "--> Saving was unsuccessful"
        n = user_input.find(":")
        marker_id = int(user_input[0:n])
        parameter_list = user_input[n+2:].split()
        
        # check if the user provided the marker and position values in the right format
        if len(parameter_list) == 3:
            rot_matrix = R.from_euler('xyz', [90.0, 0, float(parameter_list[2])], degrees=True)
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

            # check if the marker is already known to the robot, and create the proper dictonary format of a marker
            [is_existing, marker_dict] = set_ltp_aruco.create_marker_dict(marker_id, marker_pose)
            is_deleted = False
            
            if is_existing:
                # delete the old marker if it already exists
                req = DeleteResource.Request()
                resource_type = ResourceType()
                resource_type.type = ResourceType.MARKER
                req.uuid = str(marker_dict["id"])
                req.type = resource_type
                is_deleted, response = set_ltp_aruco.delete_information.send_request(req)
            if ( not is_existing or (is_existing and is_deleted) ) and len(marker_dict) > 0:
                # save the new marker in case it did not existed yet, or in case it did case but was successfully deleted
                req = SetResource.Request()
                resource_dict = {"type": "marker", "resource": marker_dict}
                req.json = json.dumps(resource_dict)
                is_called, response = set_ltp_aruco.set_information.send_request(req)
                if is_called:
                    if response.result.success:
                        feedback = "--> Successfully saved marker"
                    else:
                        feedback = f"--> Could not save marker: {response.result.message}"
            else:
                feedback = "--> Could not udpate marker, not all prior marker info could be acquired"
            print(feedback)
        else:
            print("--> Unknown user input")
        
        user_input = input("\nWhat new Aruco marker would you like to set (or type 'quit' to stop): ")
        
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    set_ltp_aruco.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()
