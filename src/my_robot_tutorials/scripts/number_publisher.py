#!/usr/bin/env python3

import random
import rospy
from std_msgs.msg import Int64

if __name__ == "__main__":
    # anonymous will add a random number after the name of the node. Therefore we can run multiple 
    # instances of this node
    rospy.init_node("number_publisher", anonymous=True)

    pub = rospy.Publisher("/number", Int64, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = Int64()
        msg.data = random.randint(0, 20)
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Node was stopped")
    