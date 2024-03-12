#include <ros/ros.h>
#include <rospy_tutorials/AddTwoInts.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_two_ints_client");
    ros::NodeHandle nh;
    ROS_INFO("Add two ints client started");

    //                                             type or request             name of service 
    ros::ServiceClient client = nh.serviceClient<rospy_tutorials::AddTwoInts>("/add_two_ints");

    // Wait until there is a server that serves the desired service
    // -1: means wait indefinitely
    // 1 means wait for 1 second
    // 2 means wait for 2 seconds, etc
    ros::service::waitForService("/add_two_ints", ros::Duration(-1));

    // Define the service and its request
    rospy_tutorials::AddTwoInts srv;
    srv.request.a = 12;
    srv.request.b = 5;

    if (client.call(srv)) {  //If service call is succesful
        ROS_INFO("Sum is %d", (int)srv.response.sum);
    }
    else {
        ROS_WARN("Service Call failed");
    }

}