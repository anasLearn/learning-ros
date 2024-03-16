#include <random>
#include  <ros/ros.h>
#include <std_msgs/Int64.h>


int main(int argc, char **argv) 
{
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, 20);

    // REgister Node. The node is anonymous
    ros::init(argc, argv, "number_publisher", ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::Int64>("/number", 10);

    ROS_INFO("Publishing numbers from a node written with C++");

    // Get the frequency form ROS params
    double publish_frequency;
    // Get the ROS param and pack it in the variable publish_frequency
    nh.getParam("/number_publish_frequency", publish_frequency);

    nh.setParam("/a_param_from_cpp", "ztx");
    
    ros::Rate rate(3);

    while(ros::ok()) {
        // Generate random number
        int random_number = dis(gen);
        std_msgs::Int64 msg;        
        msg.data = random_number;
        pub.publish(msg);
        rate.sleep();
    }
}
