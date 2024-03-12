#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    # anonymous will add a random number after the name of the node. Therefore we can run multiple 
    # instances of this node
    rospy.init_node("robot_news_radio_transmitter", anonymous=True)

    pub = rospy.Publisher("/robot_news_radio", String, queue_size=10)

    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = "Hi, this is A3m from the Robot news radio!"
        pub.publish(msg)
        rate.sleep()

    rospy.loginfo("Node was stopped")
    