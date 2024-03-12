#include <ros/ros.h>
#include <std_msgs/String.h>

void callback_receive_radio_data(const std_msgs::String& msg)
{
    ROS_INFO("Message Received: %s", msg.data.c_str());
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "smartphone_node");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/robot_news_radio", 1000, callback_receive_radio_data);

    // Keep the node running until we send a shutdown signal to it
    ros::spin();
}