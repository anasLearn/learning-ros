#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from my_robot_msgs.srv import SetLed


leds = {
    1: True,
    2: False,
    3: False
}


def handle_set_led(req):
    global leds
    if req.led_number not in range(1, 4) or req.state not in [0, 1]:
        rospy.logerr("Wrong SetLed request. LEDs stay unchanged")
        return False
    
    leds[req.led_number] = req.state == 1
    rospy.logwarn("Service called. Success")
    return True


if __name__ == "__main__":
    # anonymous will add a random number after the name of the node. Therefore we can run multiple 
    # instances of this node
    rospy.init_node("led_panel")

    service = rospy.Service(
        "/set_led",  # Name of service
        SetLed,  # Type of request
        handle_set_led  # Handle function
    )
    rospy.loginfo("Service Server has been created")

    pub = rospy.Publisher("/3_leds_status", String, queue_size=10)

    rate = rospy.Rate(1)

    msg = String()
    while not rospy.is_shutdown():
        msg.data = "The Status of the LEDS is: " + str(leds)
        pub.publish(msg)
        rate.sleep()
