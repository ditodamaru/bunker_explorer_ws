import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import time

class DataSaver:
    def __init__(self, filename):
        self.filename = filename

    def odom_callback(self, msg):
        a = msg.pose.pose.position.x
        b = msg.pose.pose.position.y
        c = msg.pose.pose.position.z
        d = msg.pose.pose.orientation.x
        e = msg.pose.pose.orientation.y
        f = msg.pose.pose.orientation.z
        g = msg.pose.pose.orientation.w
        
        with open(self.filename, "a") as h:
            l = str(a).strip('-') + "\n" + str(b).strip('-') + "\n" + str(c).strip('-') + "\n" + str(d).strip('-') + "\n" + str(e).strip('-') + "\n" + str(f).strip('-') + "\n" + str(g).strip('-') + "\n"
            h.write(l)
        
    def gps_callback(self, msg):
        latitude = msg.latitude
        longitude = msg.longitude
        
        with open(self.filename, "a") as h:
            l = "{:.8f} {:.8f}\n".format(latitude, longitude)
            h.write(l)


def main():
    rospy.init_node('data_saver')
    
    custom_filename = rospy.get_param('~filename', 'data.txt')  # Default filename is "data.txt"
    odom_filename = "/home/andi/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_navigation/waypoint_files/" + custom_filename
    gps_filename = "/home/andi/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_navigation/waypoint_files/gps_" + custom_filename
    
    data_saver = DataSaver(odom_filename)
    
    rospy.Subscriber('/odom', Odometry, data_saver.odom_callback)
    rospy.Subscriber('/gps/fix', NavSatFix, data_saver.gps_callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
