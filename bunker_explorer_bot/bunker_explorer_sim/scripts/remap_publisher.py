#!/usr/bin/env python
import rospy
from std_msgs.msg import String

if __name__ == "__main__":
    rospy.init_node("remap_publisher_node")

    pub = rospy.Publisher("topic1", String, queue_size=10)
    rate = rospy.Rate(2)

    while not rospy.is_shutdown():
        pub.publish("Hello")
        rate.sleep()