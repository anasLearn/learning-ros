#include <ros/ros.h>
#include <std_msgs/Int64.h>


int global_counter = 0;
ros::Publisher pub;


void callback_receive_number(const std_msgs::Int64& msg)
{
    global_counter++;
    ROS_INFO("Message Received: %ld", msg.data);
    std_msgs::Int64 count_msg;        
    count_msg.data = global_counter;
    pub.publish(count_msg);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "smartphone_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/number", 1000, callback_receive_number);
    pub = nh.advertise<std_msgs::Int64>("/number_count", 10);

    // Keep the node running until we send a shutdown signal to it
    ros::spin();
}

