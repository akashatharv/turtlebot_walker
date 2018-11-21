# turtlebot_walker
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)
---
This project implements a simple walker algorithm mch like a Roomba robot using the TurtleBot package. Custom launch file is used to launch the package

## Pre-requisites
The project requires ROS kinetic, catkin, Gazebo and the TurtleBot package and it is developed on UBUNTU 16.04 LTS. 

To install ROS kinetic, please follow the tutorial on: 
http://wiki.ros.org/kinetic/Installation/Ubuntu

Gazebo is normally included with ROS installation

Catkin is normally installed with ROS, If not follow :
http://wiki.ros.org/catkin

To make a catkin workspace: 
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

To install turtlebot packages use: 
```
sudo apt-get install ros-kinetic-turtlebot-gazebo 
sudo apt-get install ros-kinetic-turtlebot-apps
sudo apt-get install ros-kinetic-turtlebot-rviz-launchers
```

## How to build
After installing all dependencies and initiating your catkin workspace, build the repository using following commands

If catkin workspace wasn't created previously
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```
Now clone the repository into your catkin workspace
```
cd ~/catkin_ws/src/
git clone https://github.com/akashatharv/turtlebot_walker
cd ..
catkin_make
```

## Running the demo using launch file
Open a new terminal and type the following commands
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_walker robot.launch
```
Press ctrl+C in the window to terminate the execution

## Using rosbag to record messages over the topics

Rosbag is used to record messages published over any ROS topic. You can launch the node as described previously and simply use
```
rosbag record -a
```
in another terminal to record data from all the topics or modify the command to record a specific topic.
To use the launch file in the repository open a new terminal and type,
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch turtlebot_walker robot.launch status:=true
```
Status is a flag which is false by default which means that rosbag recording is disabled. It can be set to true as shown in commands above to record messages over all topics except /camera/*(because if we include these topics, bag file size rises astronomically) in a bag file.

Press ctrl+C in the active window to terminate the execution
The resultant bag file can be found in the results directory of the repository
To get details regarding the newly created bag file type
```
cd <path to your repository>/results
rosbag info result.bag
```
To visualize the data recorded on the screen, first run roscore and then navigate to the above directory( if launch file is not used, navigate to /.ros directory) in a different terminal
and use rosbag play 
```
rosbag play result.bag
```
You can view all the messages in any topic of interest using
```
rostopic echo <topic name>
```
Note: Gazebo should not be running during rosbag playback
