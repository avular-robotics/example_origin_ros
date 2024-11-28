# Information examples
This package defines a single ROS2 node to illustrate how operational information that is used by the Origin, such as that of Aruco markers, can be managed by a user using the ROS2 interface. To start, let us briefly present some context of the Origin's information concepts. the robot stores prior information in a so-called *knowledge base*, and the information therein is queried by the robot (and Cerebra Studio). A piece of prior information in this knowledge base is called a Resource. For example, prior information about which Aruco markers are present in the robot's surroundings, the pose of such markers, or information about the pose of predefined waypoints and also information about paths is called a Resource. The robot makes use of these Resources when executing a job, task or behavior. 

Aruco markers that have a known position with respect to the global coordinate system of the robot are, for example, being used to estimate the position of the robot with respect to that global frame. Your Origin is shipped with 3 Avular branded Aruco markers, that have a predefined position in the global frame, being

- Aruco marker with ID = 0 is positioned at (X, Y) = (0.0, 0.0) facing towards the negavitve Y-axis of the global frame;
- Aruco marker with ID = 1 is positioned at (X, Y) = (1.0, 0.0) facing towards the negavitve Y-axis of the global frame;
- Aruco marker with ID = 2 is positioned at (X, Y) = (2.0, 0.0) facing towards the negavitve Y-axis of the global frame.

When you have multiple localization maps, each linked to a particular Aruco marker, then you may want to change the position of Aruco markers with ID 1 and 2, so that their position in the robot's knowledge base matches the position between these markers and the origin of the global frame, i.e., Marker 0. Or perhaps you want to add more Aruco markers (using online jpegs printed as 20x20cm markers) so that your robot would be able to keep track of its position when driving outside the localization map (in Path Following tasks, as GoTo Waypoint task require that start and destination are on the localization map). These are some examples showing that you, as a user, might want to manage the Aruco markers (ID and position) that are known to the robot, and for that reason we will explain the details of our ROS2 node in which you may set new or update existing Aruco markers in the knowledge base of the robot.

This information example requires the Avular Origin robot <u>and</u> its autonomous capabilities as implemented by Avular's Autopilot Inference (it will thus <u>not</u> work with the Origin One simulation).

To run the example of commanding behaviors navigate into the ROS2 workspace and run
```
source install/setup.bash
ros2 run information_examples set_aruco
```
Once the node started it will first print in the terminal that it is trying to acquire the so-called *coordinate system information* of the robot, which is necessary as it specifies where in the real world the origin of the global frame is defined. This information is automatically published by the robot once the robot finished its initialization procedure, i.e., either positioned in front of a known Aruco marker, or receiving RTK-GNSS fixes. Once the coordinate system information is received, the terminal will show print statements indicating how a user may add (or update) the Aruco markers known to the robot.

The python code of this ROS2 node is found in the 'information_examples/information_examples/set_aruco.py'. The remainder of this section explains the code in more detail.

The first part of the code, i.e.,
```
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
```
creates a subclass called SetAruco for keeping the coordinate system info in a local memory and for constructing a dictionary of the information that is related to an Aruco marker. During the initialization of this subclass this local memory for the coordinate system information is being created, as well as four service-clients each for a specific query functionality, i.e., listing information for a type of resource, getting specific information about an resource, setting information on a specific resource and deleting information from a specific resource (in this case the Resource type is an Aruco marker). Each service has a specific topic by which the clients in this node calls the server running in the information manager of the robot. Further, a subscriber is defined that triggers a callback whenever new information about the coordinate system information is published.
This callback, i.e., that is triggered when the coordinate system information is being published by the Autopilot Inference, is called 'coordinate_system_callback' and stores the information that it received in the message in the local memory ``self.coordinate_system_info`` (as long as this information does not specify an unknown system information).
Finally, the subclass also defines a functionality for creating a python dictionary capturing all the information this is needed by the knowledge base for storing it (later), i.e., 'create_marker_dict'. The input of this function are the ID and the pose of the marker (with respect to the global frame, i.e., Aruco marker 0). The output of this function is a boolean flag to indicate whether the marker is already known to the robot and the dictionary of the new marker's information, which are initialized as false and an empty dict at the start of this function. Then, if the input marker ID is proper, the function first collects all the markers that are currently present in the robot's knowledge base. These will create a list of markers, after which there is a loop running over every element of that list, i.e., every known marker. Within this loop:

- The specific details of that marker are acquired, i.e., ID and pose;
- And if the ID matches the input ID, then the marker's dictionary is being filled with that information, followed by an update of its new pose and of the coordinate system information as it is currently known to the robot,


The for-loop and and if the input ID does match any of the known markers, than the marker's dictionary is filled with the information as provided by the input, i.e., ID and pose, and the coordinate system information as it is currently known to the robot.

The functions then returns the flag and the marker's dictionary.

The fourth part of the code defines the workflow of managing this information in interaction with a user.
```
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
```
It starts by initializing the ROS2 python tools via 'rclpy.init' so that we may create the actual ROS2 node 'set_aruco' that is of the class SetAruco that we just created here directly above in the third part of the code. After that, it will wait and continuously check the incoming message of the coordinate system information if it already received any useful information, i.e., different than an ``unknown`` coordinate system information. 

Then, all preconditions of this node are met and it continues with defining some printing statements for the terminal to help the user on how to define a (new) Aruco marker. After that, it will wait until the user specifies its input, which is then stored in the local variable ``user_input``.
The next while-loop will then continuously run as long as the user did not provide the input that it wants to quit. Within the while-loop we then analyze the input of the user, which is in a string format. As the ':' in the user input separates the ID of the marker from the (pose) parameters specified for that marker, we first detect this ':' and define the part before this ':' as the 'marker_id' and the part after this ':' as the 'parameters_list' (which splits the remaining string into a list of strings separated where ever the string had a space, i.e., '12 345 67' becomes ['12', '345', '67']). The few lines after turn take this parameter_list to define the pose of the marker in the global frame, where the first parameter defines the x-position of the marker, the second parameter defines the y-position of the marker, and the third parameter (heading) is used to define the rotation of the marker as a quaternion.

REMARK: Note that we also rotate the marker frame 90 degrees over the x-axis. This is because the detection algorithm for the marker defines its z-axis out of the marker and the x-axis to the right. Therefore, to compute the orientation of the marker w.r.t. the global frame we need to rotate around the global z-axis as defined by the user and then also 90 degrees over the x-axis so that the marker's z-axis will point forward.

Once the pose of the marker is computed we call the function 'create_marker_dict', as has been explained above, to create a dictionary of the marker with all information that is required by the knowledge base, along with a bool to flag if the marker already exists in the database. The if-statement next will delete that marker, if it already exists, so that the next if-statement may actually set the marker in the knowledge base (the knowledge base may not have multiple entries of the same marker ID). Note that we need to check whether the marker was successfully deleted first, in case it already exists, and that the marker's dictionary is not empty. Then, the service is called to set the marker by first turning the dictionary into a json string. The result of that service called is then also checked on whether it was called successfully and whether the marker was set successfully in the knowledge base, after which either a success message is returned or a message stating why it was not successful.

The code then continues with asking the user for a next command.

The last few lines of this code snippet are similar to the other examples for destroying the ROS2 node properly in case it is being killed.

The fifth and final part of the code, i.e.,
```
if __name__ == '__main__':
    main()
```
is a typical python implementation of linking the main functionality of the python routine.

> Note: Other Resources, such as a waypoint, path or a polygon, can be set in the database similar as to a marker. Meaning that you will first need to specify the right dictionary of that type of Resource, and then turn it into a json before calling the same service SetResource on '/autopilot/information_manager/resource/set'. Deleting and getting a Resource is done by defining a GetResource.Request() or DeleteResource.Request() and specify the ``uuid`` and ``type`` of the Resource that you want to get or delete. You may call the service of these resources at '/autopilot/information_manager/resource/get' and '/autopilot/information_manager/resource/delete'. ote that this ``uuid`` is in internal unique identifier of the knowledge base, which you may acquire by using the ListResource as introduced above.