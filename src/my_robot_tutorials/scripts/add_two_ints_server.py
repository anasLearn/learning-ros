#!/usr/bin/env python3

import rospy
from rospy_tutorials.srv import AddTwoInts


def handle_add_two_ints(req):
    result = req.a + req.b
    rospy.loginfo("Sum of " + str(req.a) + " and " + str(req.b) + " is " + str(result))
    return result


if __name__ == "__main__":
    rospy.init_node("add_two_ints_server")
    rospy.loginfo("Add 2 ints server node created")

    service = rospy.Service(
        "/add_two_ints",  # Name of service
        AddTwoInts,  # Type of request
        handle_add_two_ints  # Handle function
    )
    rospy.loginfo("Service Server has been created")

    # Spin to keep the server alive and waiting for requests
    rospy.spin()
