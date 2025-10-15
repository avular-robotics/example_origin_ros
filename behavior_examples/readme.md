# Behavior examples
This package defines a single ROS2 node to illustrate how behaviors can be commanded to the Origin. To start, let us briefly present some context of the Origin's high-level functionalities, which are perception for obtaining a situational awareness, locomotion for moving, localization for estimating speed and position, path-planning for navigating between a start and a destination waypoint while avoiding obstacles, and a knowledge-base and information manager for managing information and reason with it.

Each of the above functionalities interact with each other using ROS2 topics, services and action, so that when called in a particular schedule the Origin will exhibit a predefined behavior. A behavior is a high-level capability that the Origin is able to perform, or execute, for example WAIT, MOVE_THROUGH_POSES, MOVE_ALONG_PATH, MOVE_IN_DOCK, MOVE_OUT_DOCK andCOVER_AREA. The example below illustrates how the first three different behaviors can be used to command the robot (the others follow a similar style of implementation, be it with a different set of Policies and Constriants that must be provided in the service request).
1. A command "Move to some global waypoint" triggers the MOVE_THROUGH_POSES behavior with a single pose, i.e., its goal pose specified in the global frame of the robot (the global origin). It starts by planning a path on the map that has been loaded by the robot (the map you would also see in Cerebra Studio), and then follows that path while avoiding small and medium sized obstacles, and possibly re-plans a new path in case the obstacle is large (after some small avoidance procedures failed). Finally, it will stop moving when the destination is reached, or when some max number of re-planning attempts were triggered. It is required that the robot has loaded a localization map of the environment.
2. A command "Move to some relative waypoint" triggers the MOVE_THROUGH_POSES behavior with a single pose, i.e., its goal pose specified in the body frame of the robot (its current position). The processing and execution steps taken by the robot are equivalent to the above "Move to some global waypoint". Also here it is required that the robot has a loaded localization map of the environment.
3. A command "Follow path to some global waypoint" starts by defining a straight path from the robot's current position to the specified global waypoint marking its destination in the global frame. It then saves that path in the knowledge base of the robot, after which it continues to executing a MOVE_ALONG_PATH behavior in which the robot moves along that path while avoiding small and sometimes medium sized obstacles. The robot will stop for large obstacles. Finally, the robot stops moving when it is either at its destination or when it was obstructed by obstacles for too long. For this behavior a map is NOT required, as long as the position of the robot remains accurate, e.g., using GPS, or when moving from one map to another (in MULTIMAPPING mode, see support.avular.com)
4. A command "Wait" implies that the robot stays at its current position for some seconds.

The concept of a behavior is a piece of information to the robot, as it informs the robot what a user would like the robot to do. For that reason, a behavior and its parameters, such as constraints and policies, are managed by the knowledge-base and the information manager. This knowledge-base processes the incoming behaviors on validity and the information manager keeps track of the list of behaviors that is to be executed. This list of behaviors is polled by a so-called behavior(tree) executor, which will take the next behavior in the list and then executes that behavior. 

The protocol for managing this list of behaviors is as follows:

- In case the behavior was *successfully* executed, then the behavior is removed from the list (and the behavior executor will automatically poll the next behavior in the list).
- In case the behavior *failed* to be executed completely, then the behavior is also removed from the list (and the behavior executor will automatically poll the next behavior in the list).
- In case the emergency stop is triggered, then all behaviors will be removed from the list.
- In case the user takes control of the robot while it is executing its list of behaviors, then the execution is continued (virtually) typically resulting in a failure of the current behavior and any next one in the list.
- In case the robot receives a request to switch its mode of operation, from DEFAULT to MAPPING or vice-versa, then the list of behaviors is emptied.

## Set (and immediately execute) behaviors
<b>This example requires the Avular Origin robot <u>and</u> its autonomous capabilities as implemented by Avular's Autopilot Inference (it will thus <u>not</u> work with the Origin One simulation).</b>

To run the example of commanding behaviors navigate into the ROS2 workspace and run
```
source install/setup.bash
ros2 run behavior_examples set_behavior
```
Once the node started it will print the options in the terminal on how a user may command one of the four options, and wait for the user input. When the user provides its input, according to the proper format, the robot will start executing the behavior that was called according to the parameters that were provided.

The python code of this ROS2 node is found in the 'behavior_examples/behavior_examples/set_behavior.py', which starts an interactive terminal session with the user calling some basic functionalities in 'behavior_examples/behavior_examples/set_behavior_base.py'. The remainder of this section explains the code of these two ROS2 python nodes in more detail by starting the "set_behavior.py" that interacts with the user.

### set_behavior.py: a ROS2 node that interacts with the user and calls a service for setting behaviors

The first part of the code, i.e.,
```
import rclpy
from rclpy.node import Node

from knowledge_base_msgs.srv import SetAndExecuteBehaviors
from behavior_examples.set_behavior_base import SetBehaviorBase
```
imports the similar tools as the navigation examples. It imports some typical ROS2 tooling called 'rclpy' and specifically imports the subclass 'Node' which provides us with all functionalities for creating, spinning and killing a ROS2 node. Further, it will import the format for triggering a behavior of the Origin, i.e., a message format for calling a service format SetAndExecuteBehaviors. The final imports are the basic functionalities on which this interactive ROS2 node extends.

The second part of the code, i.e.,
```
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
```
implements the class function SetBehavior that is started by the ROS2 node to interact with the user. It starts by initializing and starting a timer callback which will request new user input every second. Immediately after that an internal (Boolean) state is being created to keep track whether the node already requested user input in the previous second and is still awaiting an answer from the user, or whether this is not the case. The last step in the initialization is the printing of the explanations about how the user may provide his or her commands. 

Since the timer callback is defined during initialization, the next function "request_input" of this class is triggered every second. This function will check whether the input of the user is still awaited by the node and whether the "coordinate system information" has already been published by the robot (this information is required when saving paths later on). Depending on this check a function "get_input" is called to actually acquire the user input and execute the behavior that is commanded.

This third and last function "get_input" in this part of the code starts by printing a statement in the terminal to the user that it is expecting the user to provide input. Unless the user indicates that he or she wants to quit, the input of the user is decomposed into the type of behavior (the user input before the ":") and any parameters that are required for that type of behavior (all user input after the ":"). These two inputs are then used in the next function that is called, which is "create_behavior". This function is defined in the base function, and thus explained later on this page. But for now it is important that this function shall return the definition of the actual behavior that needs to be executed by the robot in order for it to meet the command that was provided by the user. Since this function "create_behavior" is an asynchronous function, it needs to be awaited. The final if-statement of this "get_input" function checks whether a behavior was created and if that is the case a request is being created (and called) to set and then execute that behavior. Note that before the service is actually called it is firstly checked whether the behavior is available or not. Also this service call to add (and then execute) a behavior is an asynchronous function implemented in the information manager of the robot and it should thus be awaited. Finally, the internal state to acquire input is set to False, so that in the next second a new request to the user is made.


The third and final part of the code, i.e.,
```
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
```
is a typical python implementation of linking the main functionality of the python routine to the creation, execution, spinning and destroying of the ROS2 node.


### set_behavior_base.py: An object class that creates a behavior in the right format

The first part of the code, i.e.,
```
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
```
implements that the proper packages are being acquired by the python routine that are used later in the code. Important to note are the different types of message and service formats that are required, i.e., CoordinateSystemInfo, PoseStamped, Behavior, Constraint, Policy, SetResource and Odometry.

The second part the of code, i.e.,
```
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
```
starts with the implementation (and initialization) of the base node for setting behaviors, called SetBehaviorBase. Note that this SetBehaviorBase is a type of a ROS2 node. The initialization starts by defining two (mutually exclusive) callback groups to avoid interference of the services that are used by either this SetBehaviorBase class of its child SetBehavior (as presented above). Also two internal variables are being defined: one to track the coordinate system information as it is used by the robot (along with its subscriber "coord_info_subscription" and callback "coordinate_system_callback"), and one to track the estimated pose of the robot (also with its subscriber "pose_subscription" and callback "pose_callback"). Finally, two service clients are initialized: one for calling the service of the information manager to set and execute a behavior, i.e., "add_behavior", and one for calling a service of the information manager to save a path (later used by the MOVE_ALONG_PATH behavior), i.e., "save_path".

The third part of the code, i.e.,
```
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

```
is a function that receives the input as provided by the user and returns the actual behavior in the format that is required by the information manager, i.e., the Behavior message format. The function starts by checking the type of behavior that is being requested. 

In case a 'move_to_global" or 'move_to_relative' is requested, then the robot only requires the goal waypoint and the subfunction "_create_behavior_move_through_poses" is being called the create that specific Behavior format for a MOVE_THROUGH_POSES behavior (which is explained hereafter). 

In case a 'follow_path_to_global' is requested, then the robot needs a path and therefore a straight path to the final waypoint is computed by the function "_compute_straight_path" (see the very and of 'set_beavior_base.py'). After that, the path and coordinate system are used by the function "_create_behavior_move_along_path" to first save the path to the robot's knowledge base and then create the specific Behavior format for a MOVE_ALONG_PATH behavior (which is also explained later on this page).

In case a 'wait' is requested, then immediately the specific behavior format for a WAIT behavior is created and returned.

Next, let us explain how a spcific MOVE_THROUGH_POSES and MOVE_ALONG_PATH behavior is created.

The behavior MOVE_THROUGH_POSES is created by the function '_create_behavior_move_through_poses', which implements the following code, i.e.,
```
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
```
The function starts by defining an empty Behavior() message format, which is then populated with the proper type, i.e., MOVE_THROUGH_POSES. After that, the goal waypoint as provided by the user if converted into a typical Pose format of ROS2, i.e., with a position and an orientation in quaternions, and it also receives the proper frame, i.e., the "map" frame indicating a global waypoint or the "base_link" frame indicating a relative waypoint. The pose is then added to the list of goal poses as the only pose in the list, and therefore the destination of the robot. Finally, as an example, also the maximum speed and the type of control policy is defined. The values pointed out here are the default values, but you may change them when desired.


The behavior MOVE_ALONG_PATH is created by the function '_create_behavior_move_along_path', which implements the following code, i.e.,
```
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
                "max_height": 10.0,
                "min_height": 0.05
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
```
The function starts by createing an empte Behavior() class in the proper format followed by the creation of an path dictionary. A path is a type of Resource for the knowledge base and requires a specific structure to tbe defined. In Python we first define this structure as a dictionairy. Also, note that the actual path is defined as a list of poses, which has also been provided to this function by its caller 'create_behavior'. This dictionary contains some typical information, such as a name and a creation date, but it also requires the coordinate system information, as the (x,y) points of the path are defined with respect to the global frame that is defined in the coordinate system information. Further note the constraints defined for this path, which do not have any meaning yet although we foresee in future that each path may have a minimal and maximum speed or height.

The function then continues with saving the path to the knowledge base of the robot for which it uses the service 'save_path' with the format SetResource, as the path is a type of Resource to the knowledge base. The service call is made when the service is also available. The results of this service call are checked and if the service does not exist or in case it was not successfull, then no MOVE_ALONG_PATH should be commanded and instead we reverted to a waiting behavior. In case the path is successfully saved, then the function continues with creating the MOVE_ALONG_PATH behavior with a maximum speed and a maximum distance for the robot to the path. Also the controller can be specified as a policy, similar as to the other behavior MOVE_THOUGH_POSES. Please note that the choice of controller will also effect the strictness of the robot in following the path (selecting FollowPathLoosely will neglex the maximum path distance). Also note that the path is not provided to the robot in this MOVE_ALONG_PATH behavior message but instead the path is defined with the unique identifier that was provieded in the return of the service call for saving the path.

That was it. you should now be ready to create your own implementation for calling behaviors.