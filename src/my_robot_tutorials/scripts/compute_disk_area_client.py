#!/usr/bin/env python3

import rospy
from my_robot_msgs.srv import ComputeDiskArea


if __name__ == "__main__":
    rospy.init_node("compute_disk_area_client")
    rospy.loginfo("Compute disk area client created")

    # The code will wait here until the desired service is created
    rospy.wait_for_service("/computer_disk_area")

    try:
        # Create a function that can send requests to the service desired
        compute_disk_area = rospy.ServiceProxy(
            "/computer_disk_area",  # Service Name
            ComputeDiskArea  # Service (or request type)
        )
        response = compute_disk_area(7)
        rospy.loginfo("Are is : " + str(response.area))
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))
