#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int64
from std_srvs.srv import SetBool


class NumberCounter:
    def __init__(self) -> None:
        self.counter = 0
        self.number_subscriber = rospy.Subscriber("/number", Int64, self.callback_receive_number)
        self.pub = rospy.Publisher("/number_count", Int64, queue_size=10)
        self.reset_service = rospy.Service(
            "/reset_counter",  # Name of service
            SetBool,  # Type of request
            self.handle_reset_counter  # Handle function
        )

    def callback_receive_number(self, msg):
        rospy.loginfo("Message received: " + str(msg.data))
        self.counter += msg.data
        new_msg = Int64()
        new_msg.data = self.counter
        self.pub.publish(new_msg)

    def handle_reset_counter(self, req):
        if req.data:
            self.counter = 0    
            rospy.logwarn("Counter was reset")
            return True, "Counter was resetted after service call"
        rospy.logwarn("Service called but counter was not reset")
        return True, "Counter was not reset since the request data was false"
    
if __name__ == "__main__":
    rospy.init_node("number_counter", anonymous=True)
    NumberCounter()
    rospy.spin()
