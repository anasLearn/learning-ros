#!/usr/bin/env python3

import rospy
from my_robot_msgs.srv import SetLed


def set_led_routine(on):
    try:
        # Create a function that can send requests to the service desired
        set_led = rospy.ServiceProxy(
            "/set_led",  # Service Name
            SetLed  # Service (or request type)
        )
        response = set_led(3, on)
        rospy.loginfo("LED 3 set to: " + str(on == 1))
    except rospy.ServiceException as e:
        rospy.logwarn("Service failed: " + str(e))


if __name__ == "__main__":
    rospy.init_node("battery_node")

    # The code will wait here until the desired service is created
    rospy.wait_for_service("/set_led")

    while not rospy.is_shutdown():
        rospy.sleep(7)
        set_led_routine(1)
        rospy.sleep(3)
        set_led_routine(0)
          