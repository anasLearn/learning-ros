#!/usr/bin/env python3

from math import pi as M_PI
import rospy
from my_robot_msgs.srv import ComputeDiskArea


def handle_compute_disk_area(req):
    result = M_PI * req.radius ** 2
    rospy.loginfo("Area of circle with radius " + str(req.radius) + " = " + str(result))
    return result


if __name__ == "__main__":
    rospy.init_node("compute_disk_area_server")
    rospy.loginfo("Compute disk area server node created")

    service = rospy.Service(
        "/computer_disk_area",  # Name of service url (address)
        ComputeDiskArea,  # Type of request
        handle_compute_disk_area  # Handle function
    )
    rospy.loginfo("Service Server has been created")

    # Spin to keep the server alive and waiting for requests
    rospy.spin()
