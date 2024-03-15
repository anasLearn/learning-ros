#include <ros/ros.h>
#include <my_robot_msgs/SetLed.h>



// Define the service and its request
my_robot_msgs::SetLed srv;
    

void set_led_routine(ros::ServiceClient client, bool on){
    srv.request.led_number = 3;
    srv.request.state = on ? 1 : 0;
    if (client.call(srv)) {  //If service call is succesful
        ROS_INFO("Battery %s. Turning LED 3 %s", on ? "empty" : "full", on ? "ON" : "OFF");
    }
    else {
        ROS_WARN("Service Call failed");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "battery_node");
    ros::NodeHandle nh;
    ROS_INFO("Battery Node started");

    //                                             type or request             name of service 
    ros::ServiceClient client = nh.serviceClient<my_robot_msgs::SetLed>("/set_led");


    // Wait until there is a server that serves the desired service
    // -1: means wait indefinitely
    // 1 means wait for 1 second
    // 2 means wait for 2 seconds, etc
    ros::service::waitForService("/set_led", ros::Duration(-1));

    while(ros::ok()) {
        // Generate random number
        ros::Duration(3.0).sleep();
        set_led_routine(client, false);   
        ros::Duration(7.0).sleep();
        set_led_routine(client, true);
    }
}