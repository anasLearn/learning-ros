# ROS

This document explains some commands about ROS.
A ROS program consists of packages. Each package contains one or multiple nodes.

## Create a ROS workspace

Go to a folder of your choice (`~` for example) and create the catkin workspace.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin_make
```

Then activate the workspace by sourcing it.

```bash
cd ~/catkin_ws
source devel/setup.bash
```



## Starting the ROS Master

Always run the following command before running the nodes:

```bash
roscore
```

## ROS Nodes and Packages

Nodes are programs written in C++ or Python or another language. A ROS robot is composed of several packages. And each package contains several nodes.

Creat the Python nodes inside the folder `scripts` of a package.

Create the C++ nodes inside the folder `src` of a package.

### Create a ROS package

```bash
cd ~/catkin_ws/src
catkin_create_pkg name_of_package dependecy_1 dependency_2 dependency_etc
```

### Running a node

> **Note**
>
> When creating a node with python. always make the Python file executable using `chmod +x`. Also, it is necessary to add the line `#!/usr/bin/env python3` at the beginning of the Python file.

To run a node, follow these steps:

```bash
source devel/setup.bash
rosrun pkg_name executable_name  ## Auto-completion works
```

### Alternative way

Nodes can also be run using Python or C++. For Python nodes:

```bash
python3 path_to_python_file_directory/python_file_of_node
# or
cd path_to_python_file_directory
./python_file_of_node  # Like an executable file
```

For C++ nodes:

```bash
cd catkin_ws/devel/lib/pkg_name
./node_name
```

The node name is defined in the file `catkin_ws/src/pkg_name/CMakeLists.txt`.

### Listing Nodes

To list running nodes, execute the following command:

```bash
rosnode list
```

It will show the names of running nodes as defined in the source code of the nodes.

### To get info about a node

To get information about a specific node, use the following command:

```bash
rosnode info /node_name  ## Name as defined in the source code. You can find it in `rosnode list`
```

### To stop a node

To stop a node, use `CTRL + C` or the following command:

```bash
rosnode kill /node_name
```

### To ping a node

To ping a node, execute the following command:

```bash
rosnode ping /node_name
```

## RQT Graph

RQT graph is a tool that allows to debug the nodes and packages of a ROS robot (program).

### Visualize node with RQT graph

To visualize nodes with RQT graph, use the following command:

```bash
rosrun rqt_graph rqt_graph
```

or

```bash
rqt_graph
```

**Tip:** Uncheck debug to see all the nodes from `/rosout`.

## ROS Topics

To view all the topics available. Run this command:

```bash
rostopic list
```

To see the messages being published in a topic:

```bash
rostopic echo topic_name
```

To see the information related to a topic:

```bash
rostopic info topic_name
```

To publish a message to a topic:

```bash
rostopic pub /topic_name message_type "data: 'msg you want to send'"
## example:
rostopic pub /robot_news std_msgs/String "data: 'Hello World'"
## to publish once add -1
## to publish at a certain frequence (5 Hz for example) add -r 5
```

## ROS Services

To list services:

```bash
rosservice list
```

To call a service:

```bash
rosservice call /service_name "request content"  # You can press tab after writing the service name for autocompletion of a template request
## example:
rosservice call /add_two_ints "a: 4 
b: 5"
```

To get information about a service:

```bash
rosservice info /service_name

```

You get the node that started the service.
You also get the URI of the service. As well as its type and arguments.

**Note:** If you create 2 nodes with the same service url, the second node launched won't create that service.

## Custom Message and Service Definitons

It is better to create the defintion of custom messages and services in separate packages. Here is an example to do so:

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot_msgs roscpp rospy std_msgs
```

The definition of the messages is in the folder `msg` of the created package. Give each file the extension `.msg`.

The definition of the services is in the folder `srv` of the created package. Give each file the extension `.srv`.

Also, you need to modify Package.xml and CMakeListsts.txt for the messages to work properly (*check the video in Udemy and the my_robot_msgs package in this repo*).

When you run:

```bash
cd ~/catkin_ws
catkin_make
```

ROS generate header `.h` files in the folder `catkin_ws/devel/include/message_pkg_name`.

To be able to use this message package, you must source the catkin workspace again.

### Show Message Definition

To show the definition of a message, whether it is standard or custom, run this command:

```bash
rosmsg show message_name
```

To see a list of all the messages, run:

```bash
rosmsg list
```

### Show service defintions

**Note:** There is a difference between a service and a service definition:

> A service: is what a server is providing, it can be like this `/compute_disk_area`

and

> A service defintion: is the defintion of the content of the message that server and client send and receive. Example:
>
> ```C
> float64 radius
> ---
> float64 area 
> ```

To list all service definitions (standard and custom):

```bash
rossrv list
```

To show a particular service definition:

```bash
rossrv show service_defintion_name
```

## ROS Params

Parameters allow us to configure the project. It's like the environment variables of the projects. They can be accessed and modified from all the nodes.

To list the params:

```bash
rosparam list
```

To create or modify a ros parameter:

```bash
rosparam set /robot_name "my_robot"
rosparam set /simulation_mode false
rosparam set /sensors_read_frequency 40
```

To check the value of a ros parameter:

```bash
rosparam get /robot_name
# my_robot
```

### Notes

Changing a ROS parameter doesn't require re-running `catkin_make`.

Everytime the ROS master is stopped, all the parameters we created are deleted.

For this reason and for the need of scalability, we create launch files.

## ROS launch files

A launch file is used to start all params and nodes necessary for our robot.

It is possible to create a launch file in any package we want. But we will create a new package dedicated for the launch file:

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot_bringup
cd ..
catkin_make  # For catkin to recognize the new package
```

Create the launch file inside a folder launch inside the launch package just created (in this case `my_robot_bringup`).

```bash
cd ~/catkin_ws/src/my_robot_bringup
mkdir launch
touch my_app.launch
```

### How to launch the ROS launch file

```bash
roscore  # Launch the ROS master first
# roslaunch     pkg_name    launch_file
roslaunch my_robot_bringup my_app.launch
```

**Note:** Running `roscore` is optional. That's because `roslaunch` checks if there is a ROS master running. If there isn't, it starts one automatically. In this case, the ROS master dies when `roslaunch` is killed.

## ROS Bags

ROS Bags allows you to record and store some messages that are published to a certain topic.  And then, those same messages can be replayed anytime by being published to the topic in question.

To record a topic (record the messages sent to the topic):

```bash
rosbag record topic_name
```

When you kill `rosbag record` with `CTRL + C` a `.bag` files is created in the current directory.

To get info about the bag:

```bash
rosbag info file.bag
```

To replay the messages in the bag:

```bash
rosbag play file.bag
# The message will be then published to the topic(s) in question
```
