## Getting started
This README explains how to connect to the physical robot. You do not need to perform these steps if you use the simulation environment.
The Avular Origin has a build-in WiFi access-point and an ethernet port on its back. You therefore have two options to connect the Origin to your own computer:
1. By connecting your computer to the WiFi network of the robot, which is typically called `orgin-x-AP`, where x being the number of your Origin.
2. By mounting a companion PC on the back of the Origin, such as a Raspberry-PI, and connect a TCP cable between the Origin and your companion PC.
    * Note that this option would also require a wifi connection between your computer and the companion PC for you to SSH into the companion PC.
    * Note that this option also implies that all communication between your code (running on the companion PC) and the robot is most stable as there is a wired and not a wireless connection between the two. Which can be useful when bulky sensor data, such as LiDAR messages and camera images, and velocity commands are exchanged at high rates.

The ROS2 network of the robot is defined as a local network on the robot. This is to ensure that communication from a software of a user is not effecting the performance of the robot itself (up to some point). To make a connection with this local ROS2 network we make use of a so-called zenoh-bridge. The one side of the zenoh-bridge is already running in the robot's local ROS2 network. The other end of the zenoh-brigde should be running on your computer. Once that zenoh-bridge is up and running your computer has access to the ROS2 topics and services of the Origin. To setup this zenoh-bridge please take the following steps:
1. Install CycloneDDS  
CycloneDDS is an alternative to the default ROS2 implementation being FastDDS. CycloneDDS was found to be provide a much more stable communication when the number of nodes and messages increase. You may install the CycloneDDS middleware via `sudo apt install ros-*distro*-rmw-cyclonedds-cpp`
2. Export CycloneDDS\
Exporting CycloneDDS to your bash script ensures that it is automatically being used when opening a new terminal. You may do so by adding `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` to your .bashrc. When using docker you may use the environmental variables option when starting the docker image.
3. Instal the Zenoh bridge\
The Zenoh bridge is used to seperate the ROS2 network of the robot, as much as possible, with the ROS2 network on your local machine. The bridge can be run as a standalone binary or in a docker container. For downloads and more detailed instructions, see [this website](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds?tab=readme-ov-file#linux-debian)
4. Start the Zenoh bridge\
To start the Zenoh bridge on you local machine you should run `zenoh-bridge-ros2dds -m client -e tcp/<robot-ip>:7447`. Note that we define a new ROS2 domain. This is not compulsory but may prevent ROS2 communication issues later on.
5. Check the connection \
Open a new terminal and run `ros2 topic list`, you should now see all nodes running on the Origin ROS network.

Now you are ready to communicate with your Origin using ROS2 messages.
