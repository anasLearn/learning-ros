#include <cmath>
#include <ros/ros.h>
#include <my_robot_msgs/ComputeDiskArea.h>


// The callback of a server must return a boolean. True: Service Succesful
bool handle_compute_disk_area(
    my_robot_msgs::ComputeDiskArea::Request &req,
    my_robot_msgs::ComputeDiskArea::Response &res
)
{
    float result = M_PI * req.radius * req.radius;
    ROS_INFO("The area of a circle with a radius %f is %f", (float)req.radius, (float)result);
    res.area = result;
    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "compute_disk_area_server");
    ros::NodeHandle nh;

    ros::ServiceServer server = nh.advertiseService(
        "/computer_disk_area",
        handle_compute_disk_area
    );

    ros::spin();
}