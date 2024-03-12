#!/usr/bin/env python3

import rospy
from rospy_tutorials.srv import AddTwoInts


if __name__ == "__main__":
    rospy.init_node("add_two_ints_client")
    rospy.loginfo("Add 2 ints client node created")

    # The code will wait here until the desired service is created
    rospy.wait_for_service("/add_two_ints")

    try:
        # Create a function that can send requests to the service desired
        add_two_ints = rospy.ServiceProxy(
            "/add_two_ints",  # Service Name
            AddTwoInts  # Service (or request type)
        )
        response = add_two_ints(2, 6)
        rospy.loginfo("Sum is : " + str(response.sum))
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))
