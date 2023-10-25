#!/usr/bin/env python

import rospy
from std_msgs.msg import Char

def main():
    rospy.init_node('keyboard_publisher', anonymous=True)
    pub = rospy.Publisher('/keyboard_input', Char, queue_size=10)
    rate = rospy.Rate(10)  # Publish at 10 Hz

    try:
        while not rospy.is_shutdown():
            key = raw_input("Enter a key (or press Ctrl+C to exit): ")  # Read keyboard input
            if key:
                pub.publish(Char(ord(key)))  # Convert character to ASCII value
            rate.sleep()
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard input node has been interrupted.")
    finally:
        rospy.loginfo("Shutting down keyboard input node.")

if __name__ == '__main__':
    main()
