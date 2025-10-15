# Information examples
This package defines a single ROS2 node to illustrate how operational information that is used by the Origin, such as the Coordinate System defining the mathematical origin of the robot and Aruco markers defining pre-marked positions for the robot, can be managed by a user using the ROS2 interface. To start, let us briefly present some context of the Origin's information concepts. The robot stores prior information in a so-called *knowledge base*, and the information therein is queried by the robot (and Cerebra Studio). A piece of prior information in this knowledge base is called a Resource. For example, prior information about which Aruco markers are present in the robot's surroundings, the pose of such markers, or information about the pose of predefined waypoints and also information about paths is called a Resource. The robot makes use of these Resources when executing a job, task or behavior. 

Aruco markers that have a known position with respect to the global coordinate system of the robot are, for example, being used to estimate the position of the robot with respect to that global frame. The mathematical origin of the robot that specifies the position of the global frame in the real world is defined by latitude and longitude values in what is called the "Coordinate System Information". Upon delivery your robot is shipped with predefined values for the latitude and longitude near Avular's office in Eindhoven, The Netherlands. It is wise to update these values with the ones relevant for your robot's environment, for which you may also use this code example. Further, your Origin is shipped with 3 Avular branded Aruco markers, that have a predefined position in the global frame, being

- Aruco marker with ID = 0 is positioned at (X, Y) = (0.0, 0.0) facing towards the negative Y-axis of the global frame;
- Aruco marker with ID = 1 is positioned at (X, Y) = (1.0, 0.0) facing towards the negative Y-axis of the global frame;
- Aruco marker with ID = 2 is positioned at (X, Y) = (2.0, 0.0) facing towards the negative Y-axis of the global frame.


When you have multiple localization maps, each linked to a particular Aruco marker, then you may want to change the position of Aruco markers with ID 1 and 2, so that their position in the robot's knowledge base matches the position between these markers and the origin of the global frame, i.e., Marker 0. Or perhaps you want to add more Aruco markers (using online jpegs printed as 20x20cm markers) so that your robot would be able to keep track of its position when driving outside the localization map (in Path Following tasks, as GoTo Waypoint task require that start and destination are on the localization map). These are some examples showing that you, as a user, might want to manage the Coordinate System used by the robot as well as the Aruco markers (ID and position) that are known to the robot, and for that reason we will explain the details of our ROS2 node in which you may set new or update existing Aruco markers in the knowledge base of the robot.

<i>Please note that when the robot is powered on outdoors, then it will use its first GPS fix to define the latitude and longitude of the Coordinate System. While when powered on indoors it will use the Coordinate System that is defined in the knowledge base of the robot</i>.

## Setting the coordinate system and setting new or updating existing Aruco markers
<b>This information example requires the Avular Origin robot <u>and</u> its autonomous capabilities as implemented by Avular's Autopilot Inference (it will thus <u>not</u> work with the Origin One simulation).</b>

To run the example of commanding behaviors navigate into the ROS2 workspace and run
```
source install/setup.bash
ros2 run information_examples set_aruco
```
Once the node started it will first pose a question in the terminal about whether you want to adjust the values of the Coordinate System, i.e., the global frame, or not. If yes, then you will be asked to set the latitude, longitude and altitude of the global frame, which you may retrieve from Google maps or from GPS measurements of the robot. If you indicate that you do not want to set a new Coordinate System, then this ROS2 node will continue with trying to acquire the Coordinate System Information of the robot, which is necessary as it specifies where in the real world the origin of the global frame is defined. This information is automatically published by the robot once the robot finished its initialization procedure, i.e., either positioned in front of a known Aruco marker, or receiving RTK-GNSS fixes. Once the coordinate system information is received, the terminal will show print statements indicating how a user may add (or update) the Aruco markers known to the robot.

The python code of this ROS2 node is found in the 'information_examples/information_examples/set_aruco.py'. The remainder of this section explains the code in more detail.

The first part of the code, i.e.,
```
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
```
imports some typical ROS2 tooling called 'rclpy' and specifically imports the subclass 'Node' which provides us with all functionalities for creating, spinning and killing a ROS2 node. Also, it imports some specific formats as they are used by the knowledge base of the Origin, i.e., a messages and services related the Resource, as well as the formal of a CoordinateSystemInfo defined as an autonomy message. The final imports are several python package called 'json', 'uuid', 'numpy' and 'scipy', which you need to install on your local machine as they will be used to transform json formatted strings into python dictionaries, or for their algebraic functions.

The second part of the code, i.e.,
```
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
```
creates a subclass for a general service-client. It is an extension of service-client one may find in a ROS2 tutorial. The first routine will create the client-service as an internal node. The second routine implements the actual request that the service-client should make. Firstly, it will try to access the server side of the service and check whether it is available. If it failed, then the server of the service is not available and the service-client is finished. However, if the server is available, then the actual call is made to the server. To avoid deadlocks the service call is an asynchronous call and the client-node calling that service will keep on spinning until the service call is completed. Then, it will notify via the logger that the service call is done and return the result as it is provided by the server back to the client.

The third part of the code, i.e.,
```
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
```
starts by initializing a subclass called SetAruco for keeping the coordinate system info in a local memory and for constructing a dictionary of the information that is related to an Aruco marker. During the initialization of this subclass this local memory for the coordinate system information is being created, as well as four service-clients each for a specific query functionality, i.e., listing information for a type of resource, getting specific information about an resource, setting information on a specific resource and deleting information from a specific resource (in this case the Resource type is an Aruco marker). Each service has a specific topic by which the clients in this node calls the server running in the information manager of the robot. Further, a subscriber is defined that triggers a callback whenever new information about the coordinate system information is published.
This callback, i.e., "coordinate_system_callback", is triggered when the coordinate system information is published by the Autopilot Inference and stores that information in the local memory ``self.coordinate_system_info`` (as long as this information does not specify an unknown system information).
The next function, i.e., "print_all_markers", is a function that will print the information about all known Aruco markers in the terminal. The function starts by calling a service to list all markers that are present in the knowledge base of the robot. It will then continue with each item in that list and call a service to request all information that there is about that marker in the knowledge base. This information is the ID of the Aruco marker, its pose and the coordinate system information from which this pose is defined. It will then continue with either printing that information, when it was successfully obtained from the knowledge base, or print that the information could not be acquired.
The last function in this part, i.e., "create_marker_dict", implements a functionality for creating a python dictionary capturing all the information this is needed by the knowledge base for storing the Aruco markers and its information. The input of this function are the ID and the pose of the marker (with respect to the global frame, i.e., the coordinate system information). The output of this function is a boolean flag to indicate whether the marker is already known to the robot and the dictionary of the new marker's information, which are initialized as false and an empty dict at the start of this function. The function then continues by acquiring a list of all markers present in the knowledge base and then continues to acquire the actual information about each marker item in that list as a dictionary. In case the marker item is equal to the input marker of which the user wants to set the position, then the dictionary is updated with the desired values and a boolean was set to true to indicate that the marker is already present in the knowledge base. If no matching marker can be found, then a new marker dictionary is created. The functions then returns the boolean indicating whether the marker ID was already present in the knowledge base and it will also return the marker's dictionary.

The fourth part of the code defines the workflow of managing this information in interaction with a user. This parts start with
```
def main(args=None):
    rclpy.init(args=args)

    set_ltp_aruco = SetLtpAndAruco()
```
And finishes with 
```
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)    
    set_ltp_aruco.destroy_node()
    rclpy.shutdown()
```
This start and finish is a typical ROS2 implementation for creating a ROS2 node, in this case set_aruco, and then destroying that node when it is no longer needed. Detailed explanations can be found on the other examples of this repositories, such as the [navigation examples](../navigation_examples/readme.md). Here, let us continue with the two remaining parts that define this main function.

The first part of this main function interacts with the user about whether or not the Coordinate System Information should be updated.
```
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
```
In this first part of the main function we start with printing a Hello to the user and then immediately enter a while loop which makes a request for input to the user about whether or not the user wants to update the values of the Coordinate System, i.e., the latitude, longitude and altitude of the global frame. This while loop acquires the input from the user and checks if the input is either a 'y' or a 'n', while if none of the two were provided then the loop starts over.
In case of a yes ('y'), then the user wants to update the values of the coordinate system information. Therefore, some print statements are provided to help the user how he or she should provide these value, after which the actual input request to the user is made. Also this part is implemented as a while loop, to ensure that the user inputs 3 values. Once these 3 values are properly obtained, then they are stored in a specific message for setting the coordinate system information, i.e., "set_coord_system.send_request(req)", as the latitude, longitude and altitude of the updated coordinate system. The service of the robot to update this information is then called, and feedback is printed to the user about whether or not the coordinate system was successfully adjusted.
In case of a no ('n'), then 



The second part of this main function interacts with the user about whether or not some Aruco markers should be updated.
```
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
```
In this second part of the main function we start with printing all Aruco markers, and their detailed information on ID, pose and Coordinate System, that are currently present in the knowledge base of the robot. This way a user may check whether the Aruco marker already has the proper position in the real world and whether or not it should even be updated.
Then, it continues with printing some statements in the terminal to help the user on how to define a (new) Aruco marker. It will wait until the user specifies its input, which is then stored in the local variable ``user_input``.
The next while-loop will then continuously run as long as the user did not provide the input that it wants to quit. Within the while-loop we then analyze the input of the user, which is in a string format. As the ':' in the user input separates the ID of the marker from the (pose) parameters specified for that marker, we first detect this ':' and define the part before this ':' as the 'marker_id' and the part after this ':' as the 'parameters_list' (which splits the remaining string into a list of strings separated where ever the string had a space, i.e., '12 345 67' becomes ['12', '345', '67']). The few lines after turn take this parameter_list to define the pose of the marker in the global frame, where the first parameter defines the x-position of the marker, the second parameter defines the y-position of the marker, and the third parameter (heading) is used to define the rotation of the marker as a quaternion.

REMARK: Note that we also rotate the marker frame 90 degrees over the x-axis. This is because the detection algorithm for the marker defines its z-axis out of the marker and the x-axis to the right. Therefore, to compute the orientation of the marker w.r.t. the global frame we need to rotate around the global z-axis as defined by the user and then also 90 degrees over the x-axis so that the marker's z-axis will point forward.

Once the pose of the marker is computed we call the function 'create_marker_dict', as has been explained above, to create a dictionary of the marker with all information that is required by the knowledge base, along with a bool to flag if the marker already exists in the database. The if-statement next will delete that marker, if it already exists, so that the next if-statement may actually set the marker in the knowledge base (the knowledge base may not have multiple entries of the same marker ID). Note that we need to check whether the marker was successfully deleted first, in case it already exists, and that the marker's dictionary is not empty. Then, the service is called to set the marker by first turning the dictionary into a json string. The result of that service called is then also checked on whether it was called successfully and whether the marker was set successfully in the knowledge base, after which either a success message is returned or a message stating why it was not successful.

The code then continues with asking the user for a next command.

The fifth and final part of the code, i.e.,
```
if __name__ == '__main__':
    main()
```
is a typical python implementation of linking the main functionality of the python routine.

> Note: Other Resources, such as a waypoint, path or a polygon, can be set in the database similar as to a marker. Meaning that you will first need to specify the right dictionary of that type of Resource, and then turn it into a json before calling the same service SetResource on '/autopilot/information_manager/resource/set'. Deleting and getting a Resource is done by defining a GetResource.Request() or DeleteResource.Request() and specify the ``uuid`` and ``type`` of the Resource that you want to get or delete. You may call the service of these resources at '/autopilot/information_manager/resource/get' and '/autopilot/information_manager/resource/delete'. ote that this ``uuid`` is in internal unique identifier of the knowledge base, which you may acquire by using the ListResource as introduced above.