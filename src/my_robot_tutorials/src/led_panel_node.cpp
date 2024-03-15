#include <ros/ros.h>
#include <std_msgs/String.h>
#include <my_robot_msgs/SetLed.h>


std::vector<bool> leds = {true, false, false};


// The callback of a server must return a boolean. True: Service Succesful
bool handle_set_led(
    my_robot_msgs::SetLed::Request &req,
    my_robot_msgs::SetLed::Response &res
)
{
    if (req.led_number < 1 || req.led_number > 3 || (req.state != 0 && req.state != 1)) {
        ROS_ERROR("Wrong SetLed request. LEDs stay unchanged");
        res.success = false;
        return true;
    }
    
    leds[req.led_number - 1] = (req.state == 1);
    ROS_WARN("Service called. Success");
    res.success = true;
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "led_panel_node");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService(
        "/set_led",
        handle_set_led
    );

    ros::Rate rate(1);

    ros::Publisher pub = nh.advertise<std_msgs::String>("/3_leds_status", 10);

    while(ros::ok()) {
        std_msgs::String msg;
        msg.data = "All LEDs status: LED 1: " + std::string(leds[0] ? "true" : "false") +
           ". LED 2: " + std::string(leds[1] ? "true" : "false") +
           ". LED 3: " + std::string(leds[2] ? "true" : "false");
        pub.publish(msg);

        ros::spinOnce(); // Handle callbacks

        rate.sleep();
    }
}