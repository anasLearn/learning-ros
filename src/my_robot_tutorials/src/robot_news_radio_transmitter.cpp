#include  <ros/ros.h>
#include <std_msgs/String.h>

int main(int argc, char **argv) 
{
    // The node is anonymous
    ros::init(argc, argv, "robot_news_radio_transmitter", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("/robot_news_radio", 10);

    ROS_INFO("Radio transmission started");

    ros::Rate rate(3);

    while(ros::ok()) {
        std_msgs::String msg;
        msg.data = "Hi, this is Bear from robot news radio!";
        pub.publish(msg);
        rate.sleep();
    }
}