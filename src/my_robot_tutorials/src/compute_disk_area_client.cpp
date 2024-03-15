#include <ros/ros.h>
#include <my_robot_msgs/ComputeDiskArea.h>


// The callback of a server must return a boolean. True: Service Succesful
int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_client", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ROS_INFO("Compute Disk Area client started");

    //                                             type or request             name of service address
    ros::ServiceClient client = nh.serviceClient<my_robot_msgs::ComputeDiskArea>("/computer_disk_area");

    // Wait until there is a server that serves the desired service
    // -1: means wait indefinitely
    // 1 means wait for 1 second
    // 2 means wait for 2 seconds, etc
    ros::service::waitForService("/computer_disk_area", ros::Duration(-1));

    // Define the service and its request
    my_robot_msgs::ComputeDiskArea srv;
    srv.request.radius = 12;

    if (client.call(srv)) {  //If service call is succesful
        ROS_INFO("Area is %f", (float)srv.response.area);
    }
    else {
        ROS_WARN("Service Call failed");
    }
}
