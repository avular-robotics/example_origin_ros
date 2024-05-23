# HelloOrigin
Welcome to the repository of code-examples for the Avular Origin

In this repository you will learn the basics to interact with the robot using ROS2. This means that background knowledge on ROS2 is a must. Also, because our robot runs ROS2 Humble, we require for the examples in this repository that you run the code from a computer (or docker-image) in which ROS2-Humble is installed. If you don't have ROS2 Humble installed, please have a look on the [ROS2 install guide](https://docs.ros.org/en/humble/Installation.html), while if you have the feeling that you lack some background knowledge on ROS2, please have a look on the [ROS2 Tutorials](https://docs.ros.org/en/humble/Tutorials.html). Also, since our examples are written in Python, you may focus on the Python examples of the ROS2 tutorials.

## Outline of the code example
1. [Short introduction on getting started](src.readme.md)
2. [Examples to acquire the position and velocity](src/navigation_examples/readme.md)
3. [Examples to set the velocity](src/navigation_examples/readme.md)
4. [Examples to set a behavior](src/behavior_examples/readme.md)

## Setting up
The Origin supports a ROS2 interface that you may use to interact your local machine with your Origin, for example using your laptop or a companion PC (Raspberry PI 4/5). The interface is made in ROS2 Humble, so you are required to either run ROS2 Humble on you local machine as well. An alternative would be to run a docker container. There are multiple images for ROS2 Humble, depending on how many packages you want to have pre-installed automatically. Since you will be developing, I would advise you to pull the 'desktop' or the 'perception' version of Humble rather than its 'core' or 'base' version. Assuming that you have installed en setup docker, you may pull one of the following images. 
```
docker pull osrf/ros:humble-desktop
```
or
```
docker pull osrf/ros:humble-perception
```
Before you continue with setting up the code example of this repository, please read this [short introduction]((src.readme.md) on how to setup the ROS2 interface between the Origin and your local machine.


Now you are ready to setup this repository as a ROS2 workspace on your machine (or inside the docker container). This is simlar to the examples in the ROS2 tutorial on [creating your own package](https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html), with some changes on where to copy this repository and install some dependencies.

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


In the last two steps you will install some dependencies, such as the nav2 messages of the [ROS2 navigation stack](https://navigation.ros.org/), the geomtry and sensor messages of ROS2 that should come with you basic install of ROS2 Humble, and some frequently used Python packages being [numpy](https://numpy.org/) and [scipy](https://scipy.org/).
```
sudo apt install ros-humble-nav2-msgs
pip3 install numpy scipy
```
At this moment you are ready to start the examples of this repository. To do so you must first build the ROS2 packages in this workspace. Navigate into the top folder "HelloOrigin' and run the following command
```
colcon build --symlink-install
```
The argument '--symlink-install' will allow you to modify python-code without the need to rebuild the workspace.

The prefered order of examples is to start with the [navigation_examples](src/navigation_examples/readme.md), in which you will echo velocity and position and set velocity. After that you may continue to the [perception_examples](src/perception_examples/readme.md) on getting the closest obstruction in the vicinity of the Origin. The [behavior_examples](src/behavior_examples/readme.md) are the last examples of this tutorial, in which you will learn how to sent a command to the Origin for it to execute a behavior, such as a move to or a wait.

After these three basic examples you should be able to create you own ROS2-packages that interface with the robot, in which you can develop you own application.

Good luck!
And have fun!
