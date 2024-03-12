#!/usr/bin/env python3


import rospy
from std_msgs.msg import Int64


num_counter = 0
pub = None

def callback_receive_number(msg):
    global num_counter
    num_counter += msg.data
    new_msg = Int64()
    new_msg.data = num_counter
    pub.publish(new_msg)


if __name__ == "__main__":
    # anonymous will add a random number after the name of the node. Therefore we can run multiple 
    # instances of this node
    rospy.init_node("number_counter", anonymous=True)

    
    sub = sub = rospy.Subscriber("/number", Int64, callback_receive_number)
    pub = rospy.Publisher("/number_count", Int64, queue_size=10)

    # Keep the script here without going further
    rospy.spin()

    