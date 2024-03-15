# ROS

This document explains some commands about ROS.
A ROS program consists of packages. Each package contains one or multiple nodes.

## Create a ROS workspace

Go to a folder of your choice (`~` for example) and create the catkin workspace.

```bash
mkdir -p ~/catkin_ws/src
```

Then activate the workspace by sourcing it.

```bash
cd ~/catkin_ws
source devel/setup.bash
```

## Create a ROS package

```bash
cd ~/catkin_ws/src
catkin_create_pkg name_of_package dependecy_1 dependency_2 dependency_etc
```

## Starting the ROS Master

Always run the following command before running the nodes:

```bash
roscore
```

## Running a node

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

## Listing Nodes

To list running nodes, execute the following command:

```bash
rosnode list
```

It will show the names of running nodes as defined in the source code of the nodes.

## To get info about a node

To get information about a specific node, use the following command:

```bash
rosnode info /node_name  ## Name as defined in the source code. You can find it in `rosnode list`
```

## To stop a node

To stop a node, use `CTRL + C` or the following command:

```bash
rosnode kill /node_name
```

## To ping a node

To ping a node, execute the following command:

```bash
rosnode ping /node_name
```

## Visualize node with RQT graph

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

## Custom Message Definitons

It is better to create the defintion of custom messages in separate packages. Here is an example to do so:

```bash
cd ~/catkin_ws/src
catkin_create_pkg my_robot_msgs roscpp rospy std_msgs
```

The definition of the messages is in the folder `msg` of the created package. Give each file the extension `.msg`.

Also, you need to modify Package.xml and CMakeListsts.txt for the messages to work properly.

When you run:

```bash
cd ~/catkin_ws
catkin_make
```

ROS generate a header `.h` file in the folder `catkin_ws/devel/include/message_pkg_name`.

To be able to use this message package, you must source the catkin workspace again.

## Show Message Definition

To show the definition of a message, whether it is standard or custom, run this command:

```bash
rosmsg show message_name
```

To see a list of all the messages, run:

```bash
rosmsg list
```

## Show service defintions

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
