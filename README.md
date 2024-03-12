# ROS

This document explains some commands about ROS.

## Starting the ROS Master

Always run the following command before running the nodes:

```bash
roscore
```

## Running a node

To run a node, follow these steps:

```bash
source devel/setup.bash
rosrun pkg_name executable_name  ## Auto-completion works
```

### Alternative way

Nodes can also be run using Python or C++. For Python nodes:

```bash
python3 path_to_python_file_directory/python_file_of_node
```

For C++ nodes:

```bash
cd catkin_ws/devel/lib/pkg_name/node_name
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
