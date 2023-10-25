#!/usr/bin/python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
#from teleop_twist_keyboard.msg import TeleopTwist
import time
#import keyboard
from std_msgs.msg import String  # Import String message type

class DataSaver:
    def __init__(self, filename):
        self.filename = filename
        self.robot_position = None
        self.gps_data = None

    def odom_callback(self, msg):
        self.robot_position = msg.pose.pose

    def gps_callback(self, msg):
        self.gps_data = msg

    def save_data(self):
        if self.robot_position is None or self.gps_data is None:
            return

        with open(self.filename, "a") as h:
            robot_pose = self.robot_position
            a = robot_pose.position.x
            b = robot_pose.position.y
            c = robot_pose.position.z
            d = robot_pose.orientation.x
            e = robot_pose.orientation.y
            f = robot_pose.orientation.z
            g = robot_pose.orientation.w
            l = str(a).strip('-') + "\n" + str(b).strip('-') + "\n" + str(c).strip('-') + "\n" + str(d).strip('-') + "\n" + str(e).strip('-') + "\n" + str(f).strip('-') + "\n" + str(g).strip('-') + "\n"
            h.write(l)

        with open(self.gps_filename, "a") as h:
            latitude = self.gps_data.latitude
            longitude = self.gps_data.longitude
            gps_data_str = "{:.8f} {:.8f}\n".format(latitude, longitude)
            h.write(gps_data_str)

def key_callback(msg, data_saver):
    if msg.data == 's':
        data_saver.save_data()
        print("Data saved.")    

def main():
    rospy.init_node('data_saver')
    
    custom_filename = rospy.get_param('~filename', 'data.txt')
    odom_filename = "/home/andi/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_navigation/waypoint_files/" + custom_filename
    gps_filename = "/home/andi/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_navigation/waypoint_files/gps_" + custom_filename
    
    data_saver = DataSaver(odom_filename)
    data_saver.gps_filename = gps_filename
    
    rospy.Subscriber('/odom', Odometry, data_saver.odom_callback)
    rospy.Subscriber('/ublox_gps/fix', NavSatFix, data_saver.gps_callback)
    rospy.Subscriber('/keyboard_input', String, key_callback, callback_args=data_saver)  # Subscribe to keyboard input

    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        if keyboard.is_pressed('s'):
            data_saver.save_data()
            print("Data saved.")
            time.sleep(1)  # Delay to prevent multiple saves on a single key press
            
        rate.sleep()

if __name__ == '__main__':
    main()
