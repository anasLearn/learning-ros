#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool


num_counter = 0
pub = None

def callback_receive_number(msg):
    global num_counter
    rospy.loginfo("Message received: " + str(msg.data))
    num_counter += msg.data
    new_msg = Int64()
    new_msg.data = num_counter
    pub.publish(new_msg)

def handle_reset_counter(req):
    global num_counter
    if req.data:
        num_counter = 0    
        rospy.logwarn("Counter was reset")
        return True, "Counter was resetted after service call"
    rospy.logwarn("Service called but counter was not reset")
    return True, "Counter was not reset since the request data was false"


if __name__ == "__main__":
    # anonymous will add a random number after the name of the node. Therefore we can run multiple 
    # instances of this node
    rospy.init_node("number_counter", anonymous=True)

    sub = rospy.Subscriber("/number", Int64, callback_receive_number)
    pub = rospy.Publisher("/number_count", Int64, queue_size=10)

    service = rospy.Service(
        "/reset_counter",  # Name of service
        SetBool,  # Type of request
        handle_reset_counter  # Handle function
    )
    rospy.loginfo("Service Server has been created")

    # Keep the script here without going further
    rospy.spin()
