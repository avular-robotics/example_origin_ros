# Behavior examples
This package defines a single ROS2 nodes to illustrate some behavior principles of the Origin. To start, let us briefly present some context of the Origin's high-level functionalities, which are perception for obtaining a situational awareness, locomotion for moving, localization for estimating its speed and position, path-planning for navigating between a start and a destination waypoint while avoiding obstacles, and a knowledge-base and informaiton manager for managing information and reason with it.

Each of the above functionalities have additional ROS2 topics, services and actions that, when called in a particular schedule, will exhibit a so-called behavior of the Origin. A behavior is a high-level capability that the Origin is able to perform, or execute, for example as part of a complete inspection task. The example below illustrates 3 different behaviors that a user may request to the robot for execution, which are:
1. Move to some globally reference waypoint, i.e., with respect to the global origin, which will plan a path on a map that is made available, follow that path, avoid small and medium sized obstacles that are encountered on that path, re-plan a new path in case the obstacle is large (after some small avoidance procedures failed), stop moving when the destination is reached.
2. Move to some relatively reference waypoint, i.e., with respect to the robot's current position, which will plan a path on a map that is made available, follow that path, avoid small and medium sized obstacles that are encountered on that path, re-plan a new path in case the obstacle is large (after some small avoidance procedures failed), stop moving when the destination is reached.
3. Wait for some seconds

The concept of a behavior is a piece of information to the robot as it informs the robot what a user would like the robot to do. For that reason, a behavior and its parameters, such as constraint and policy, are managed by the knowledge-base and the information manager. This knowledge-base processes the incoming behaviors on validity and the information manager keeps track of the llist of behaviors that is to be executed. Another functionality in the robot continuously polls the information manager on which behavior is to be executed next, which is then removed from the list in case the behavior was executed succesfully.

This example only requires the Avular Origin robot <u>and</u> its autonomous capabilities as implemented by Avular's Autopilot Inference (it will thus <u>not</u> work with the Origin One simulation).

To run the example of commanding behaviors navigate into the ROS2 workspace and run
```
source install/setup.bash
ros2 run behavior_examples set_behavior
```
Once the node started it will print the options in the terminal on how a user may command one of the three behaviors, and wait for the user input. When the user provides its input, according to the proper format, the robot will start executing the behavior that was called according to the parameters that were provided.

The python code of this ROS2 node is found in the 'behavior_examples/behavior_examples/set_behavior.py'. The remainder of this section explains the code in more detail.

The first part of the code, i.e.,
```
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from knowledge_base_msgs.msg import Behavior, Constraint, Policy
from knowledge_base_msgs.srv import SetAndExecuteBehaviors

#from avular_utils import ClientService
from scipy.spatial.transform import Rotation as R
import numpy as np

NR_RETRIES = 3
```
imports the similar tools as the navigation examples. It imports some typical ROS2 tooling called 'rclpy' and specifically imports the subclass 'Node' which provides us with all functionalities for creating, spinning and killing a ROS2 node. Also, it imports the 'PoseStamped' message format of the ROS2 geometry package (geometry_msgs that you have installed in addition to this workspace). Further, it will import some sepcific formats as they are used by the knowledge base of the Origin, i.e., a message formate Behavior and a service format SetAndExecuteBehaviors. You will need to import this format so that the node is able to read the incoming messages properly. The final imports are two python package called 'numpy' and 'scipy', which you also installed on your local machine and are toolboxes on Linear Algebra and goniometry, respectively. Also, we define a global parameters NR_RETRIES, which specifies how many times we retry to call a service in case it was not succesfull.

The second part of the code, i.e.,
```
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
```
creates a subclass for a general service-client. It is an extension of service-client one may find in a ROS2 tutorial. The first routine will create the client-service as an internal node. The second routine implements the actual request that the service-client should make. Firstly, it will try to access the server side of the service and check whether it is available. It will to so 3 times (NR_RETRIES) with intervals of half a second. A counter will keep track of how many attempts were made. Then, if all attempts failed, then the server of the service is not available and the service-client is finished. However, it the server is availabe, then the actual call is made to the server. To avoid deadlocks the service call is an asynchronous call and the client-node calling that service will keep on spinnig until the service call is completed. Then, it will notify via the logger that the service call is done and return the result as it is provided by the server back to the client.

The thrid part of the code, i.e.,
```
class SetBehavior(Node):

    def __init__(self):
        super().__init__('velocity_commander')
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
```
creates a subclass called SetBehavior for constructing a behavior and calling a service to add that behavior to the list of behaviors that is planned for execution. During the intialization of this subclass a new service-client is being created named 'self.add_behaviors' that is of the type ClientService that was just defined in the code snippet directly above. The client calls a service on the topic '/autopilot/information_manager/set_and_execute_behaviors', which is a service do append new behaviors to the list of behaviors that are planned for execution.
The subclass also defines a functionality for constructing a behavior, i.e., 'create_behavior', which requires a type as it is known to a user and parameters. The fuctions start by create an empty Behavior in the variable 'behavior' and an empty list of 'behaviors'. Then, depending on the type of behaviors that is being requested by the user, i.e., 'move_to_global', 'move_to_relative', or 'wait', the if-then-else-loop goes to a particular case. In case no proper type was provided the function will print in the terminal that the provided type of behavior is unknown. Then, for the different cases we have:
- case: 'move_to_global' or 'move_to_relative'
    Exract the waypoint from the parameters and check of the waypoint has 3 elements (1 for linear in x, 1 for linear in y and 1 for orientation around z). If not, then print a logging statement in the terminal that it cannot set the behavior as there are an unknown number of elements in the waypoint. If the waypoint does contain 3 elements, as expected, then define that the type of behavior as it is known to the Origin is MOVE_THROUGH_POSE. Then create an empty variable 'pose' which is of the type PoseStamped, after which the next 2 lines define the x-position and y-position of the pose according to the provided waypoint. Further, the desired heading is turned into a rotation matrix 'r' and transformed into quaternions that are used to define the orientation of the 'pose'. We will then define the frame_id of the 'pose', which defines the mathematical origin from which the pose, or waypoint, is defined. In case the user defined a 'move_to_global', then the frame_id is 'map' since the origin of the map is the global origin, while if the user defined a 'move_to_relative', then the frame_id is 'base_link' which specifies that the origin of the 'pose', or waypoint, is the Origin-robot itself. Finally, the 'pose' variable is added to the list of goals of the behavior (more than 1 is allowed but not specified here), and in the policy field of the behavior we set the maximum speed to be 1.0 m/s (anything between 0.1 and 1.5 is feasible and will be adopted by the Origin). The behavior is then added to the list of behaviors.

- case: 'wait'
    Extract the waiting time from the list of parameters, define that the type of behavior as it is known to the Origin is WAIT, add the waiting time as a constraint to the behavior, and append it to the list of behaviors
The functions returns the list of behaviors created in the correct format.

The fourth part of the code, i.e.,
```
def main(args=None):
    rclpy.init(args=args)

    set_behavior = SetBehavior()
    
    print("----------------- hello user -----------------------")
    print("define the next behavior")
    print("move_to_global: 1.0, 1.0, 90.0 ------- [m m deg]")
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
```
implements the main strategy for a user to set a behavior. It starts by intializing the ROS2 python tools via 'rclpy.init' so that we may create the actual ROS2 node 'set_behavior' that is of the class SetBehavior that we just created here directly above in the third part of the code. After that, some printing statements are defined in the terminal to help the user on how to define a next behavior, which is then stored in the local variable user_input.
The while-loop will then continuously run as long as the user did not provide the input that it wants to quit. Within the while-loop we then analyze the input of the user, which is in a string format. As the ':' in the user input seperates the type of behavior from the parameters specified for that behavior, we first detected this ':' and define the part before this ':' as the 'behavior_type' and the part after this ':' as the 'parameters_list' (which splits the remaining string into a list of strings seperated where ever the string had a space, i.e., '12 345 67' becomes ['12', '345', '67']). The few lines after turn the 'parameter_list' from a list of strings into a list of floats and then call the 'create_behavior' routine of the class set_behavior that will turn the user input into the proper format of a Behavior as it is expected by the Origin (see also the previous code snippet directly above this one).
When a non-empty list of behavior has been created and is returned, we then define an empty request service of the type SetAndExecuteBehaviors in which the list of newly created behaviors is appended and the service call to the server running in the information manager is being made. Note that the return of the service is nog being checked and the code immediately asks the user to provide the next input.
The last few lines of this code snippet are similar to the other examples for destrying the ROS2 node properly in case it is being killed.

The fifth and final part of the code, i.e.,
```
if __name__ == '__main__':
    main()
```
is a typical python implementation of linking the main functionality of the python routine.
