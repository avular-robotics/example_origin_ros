# HelloOrigin
Welcome to the repository of code-examples for the Avular Origin

In this repository you will learn the basics to interact with the robot using ROS2. This means that background knowledge on ROS2 is a must. Also, because our robot runs ROS2 Humble, we require for the examples in this repository that you run the code from a computer (or docker-image) in which ROS2-Humble is installed. If you don't have ROS2 Humble installed, please have a look on the [ROS2 install guide](https://docs.ros.org/en/humble/Installation.html), while if you have the feeling that you lack some background knowledge on ROS2, please have a look on the [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html). Also, since our examples are written in Python, you may focus on the Python examples of the ROS2 tutorials.

## Outline
1. [Short introduction on getting started](###getting-started)
2. [Examples to acquire the position and velocity](src/navigation_examples/readme.md)
3. [Examples to set the velocity](src/navigation_examples/readme.md)
4. [Examples to set a behavior](src/behavior_examples/readme.md)

### Getting started
The Avular Origin has a build-in WiFi access-point and an ethernet port on its back. You therefore have two options to connect the Origin to your own computer:
1. By connecting your computer to the WiFi network of the robot, which is typically called `orgin-x-AP`, where x being the number of your Origin.
2. By mounting a companion PC on the back of the Origin, such as a Raspberry-PI, and connect a TCP cable between the Origin and your companion PC.
    * Note that this option would also require a wifi connection between your computer and the companion PC for you to SSH into the companion PC.
    * Note that this option also implies that all communication between your code (running on the companion PC) and the robot is most stable as there is a wired and not a wireless connection between the two. Which can be usefull when bulky sendor data, such as LiDAR messages and camera images, and velocity commands are exchanged at high rates.

The ROS2 network of the robot is defined as a local network on the robot. This is to ensure that communication from a software of a user is not effecting the performance of the robot itself (up to some point). To make a connection with this local ROS2 network we make use of a so-called zenoh-bridge. The one side of the zenoh-bridge is already running in the robot's local ROS2 network. The other end of the zenoh-brigde should be running on your computer. Once that zenoh-bridge is up and running your computer has access to the ROS2 topics and services of the Origin. To setup this zenoh-bridge please take the following steps:
1. Install zenoh
2. Install CycloneDDS
3. Export CycloneDDS
4. Start the zenoh-bridge
5. check the connection

After these steps you are ready to setup this repository as a ROS2 workspace on your computer. This is simlar to the examples in the ROS2 tutorial on [creating your own package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), with some changes on where to copy this repository and install some dependencies.

First you will need to make a new folder, which will be your ROS2 workspace using the following command
```
mkdir HelloOrigin
```
Then you clone this repository into the folder as follows
```
git clone https://github.com/jorissijs/HelloOrigin.git
```
When entering the src folder you will see the following structure.
```
HelloOrigin
 |
 --- src
      |
      --- autonomy_msgs
      |
      --- behavior_examples
      |
      --- knowledge_base_msgs
      |
      --- navigation_examples
      |
      --- perception_examples
      |
      --- origin_msgs      
```
The folders that finish with '_examples' are ROS2 packages containing Python ROS2-examples related to robot behaviors, robot navigation and robot perception. The other three folders that finish with '_msgs' are ROS2 packages that define specific message formats that are used by the Origin. You will need these messages in the workspace so that the ROS2 topics and services can be recieved and processed by your computer. 


In the last two steps you will install some dependencies, such as the nav2 messages of the [ROS2 navigation stack](https://navigation.ros.org/), the geomtry and sensor messages of ROS2 that should come with you basic install of ROS2 Humble, and some frequently used Python packages being [numpy](https://numpy.org/) and [sciy](https://scipy.org/).
```
sudo apt install ros-humble-nav2-msgs
pip3 install numpy scipy
```
At this moment you are ready to start the examples of this repository. To do so you must first build the ROS2 packages in this workspace. Navigate into the top folder "HelloOrigin' and run the following command
```
colcon build --symlink-install
```
The argument '--symlink-install' will allow you to modify python-code without the need to rebuild the workspace.

The prefered order of examples is to start with the [navigation_examples]((src/navigation_examples/readme.md)), in which you will echo velocity and position and set velocity. After that you may continue to the [perception_examples](src/perception_examples/readme.md) on getting the closest obstruction in the vicinity of the Origin. The [behavior_examples](src/behavior_examples/readme.md) are the last examples of this tutorial, in which you will learn how to sent a command to the Origin for it to execute a behavior, such as a move to or a wait.

After these three basic examples you should be able to create you own ROS2-packages that interface with the robot, in which you can develop you own application.

Good luck!
And have fun!
