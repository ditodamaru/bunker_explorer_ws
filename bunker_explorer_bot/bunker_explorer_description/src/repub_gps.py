#!/usr/bin/env python
import rospy
from sensor_msgs.msg import NavSatFix

def callback(data):
    global pub
    data.altitude = 0
    pub.publish(data)
    
def listener():
    global pub
    rospy.init_node('repub', anonymous=True)
    pub = rospy.Publisher('/gps/fix2', NavSatFix, queue_size=10)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()