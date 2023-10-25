#!/usr/bin/env python

import rospy
import rospkg
import math
import sys
import std_msgs.msg
from sensor_msgs.msg import NavSatFix, Joy
from std_msgs.msg import Bool, Char
from nav_msgs.msg import Odometry  # Import the Odometry message type


collect_request = False
continue_collection = True
lati_point = 0
longi_point = 0
lati_last = 0
longi_last = 0
min_coord_change = 10 * math.pow(10, -6)
min_move_change = 0.5
end_button_sym = ""
collect_button_sym = ""
end_button_num = ""
collect_button_num = ""

# Define variables for robot position and orientation
robot_position_x = 0.0
robot_position_y = 0.0
robot_position_z = 0.0
robot_orientation_x = 0.0
robot_orientation_y = 0.0
robot_orientation_z = 0.0
robot_orientation_w = 0.0
robot_position_last_x = 0.0
robot_position_last_y = 0.0
robot_position_last_z = 0.0

# Define a debounce interval in seconds (e.g., 0.5 seconds)
debounce_interval = 0.01

# Define a timestamp to keep track of the last button press
#last_button_press_time = rospy.Time.now().to_sec()

def joy_CB(joy_msg):
    global collect_request, continue_collection
    if joy_msg.buttons[0] == ord(collect_button_num):
        collect_request = True
    else:
        collect_request = False

    if joy_msg.buttons[0] == ord(end_button_num):
        continue_collection = False

def keyboard_CB(msg):
    global collect_request, continue_collection, last_button_press_time  # Add last_button_press_time here
    key_pressed = msg.data
    #key_pressed = ord(msg.data)  # Convert received char to ASCII value
    print("Received key:", key_pressed)  # Add this line

    # Calculate the time difference since the last button press
    time_since_last_press = (rospy.Time.now() - last_button_press_time).to_sec()

    if key_pressed == 115: # ASCII value for 's'
        if time_since_last_press > debounce_interval:
            collect_request = True
            last_button_press_time = rospy.Time.now()
    elif key_pressed == 101: # ASCII value for 'e'
        continue_collection = False


def filtered_gps_CB(gps_msg):
    global lati_point, longi_point
    lati_point = gps_msg.latitude
    longi_point = gps_msg.longitude

def odom_CB(odom_msg):
    global robot_position_x, robot_position_y, robot_position_z
    global robot_orientation_x, robot_orientation_y, robot_orientation_z, robot_orientation_w

    # Extract robot position and orientation data from Odometry message
    robot_position_x = odom_msg.pose.pose.position.x
    robot_position_y = odom_msg.pose.pose.position.y
    robot_position_z = odom_msg.pose.pose.position.z
    robot_orientation_x = odom_msg.pose.pose.orientation.x
    robot_orientation_y = odom_msg.pose.pose.orientation.y
    robot_orientation_z = odom_msg.pose.pose.orientation.z
    robot_orientation_w = odom_msg.pose.pose.orientation.w


def main():
    global collect_request, continue_collection, lati_point, longi_point, lati_last, longi_last, min_coord_change
    global end_button_sym, collect_button_sym, end_button_num, collect_button_num
    global robot_position_x, robot_position_y, robot_position_z, robot_orientation_x, robot_orientation_y, robot_orientation_z, robot_orientation_w, robot_position_last_x, robot_position_last_y, robot_position_last_z

    global min_move_change

    global last_button_press_time  # Declare it as global if you want to modify it

    
    # Initialize node
    rospy.init_node("collect_slam_waypoints")

    # Initialize last_button_press_time here, after calling rospy.init_node()
    last_button_press_time = rospy.Time.now()


    # Initialize subscribers
    rospy.Subscriber("/keyboard_input", Char, keyboard_CB)
    rospy.Subscriber("/joy_teleop/joy", Joy, joy_CB)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, filtered_gps_CB)
    rospy.Subscriber("/rtabmap/odom", Odometry, odom_CB)  # Subscribe to rtabmap/odom topic
    rospy.loginfo("Initiated collect_gps_waypoints node")

    # Initialize publisher
    pub_collection_node_ended = rospy.Publisher("/outdoor_waypoint_nav/collection_status", Bool, queue_size=100)

    # Read parameters
        #coordinates_file_dir = rospy.get_param("/outdoor_waypoint_nav/coordinates_file")
    #coordinates_file_dir = rospy.get_param("coordinates_file_dir")
    coordinates_file_dir = "/home/andi/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_navigation/waypoint_files/points_slam_indoor.txt"
    #end_button_sym = rospy.get_param("/outdoor_waypoint_nav/end_button_sym")
    end_button_sym = "E"
    #collect_button_sym = rospy.get_param("/outdoor_waypoint_nav/collect_button_sym")
    collect_button_sym = "S"
    #end_button_num = rospy.get_param("/outdoor_waypoint_nav/end_button_num")
    end_button_num = "e"
    #collect_button_num = rospy.get_param("/outdoor_waypoint_nav/collect_button_num")
    collect_button_num = "s"
    rospy.loginfo("Saving coordinates to: %s", coordinates_file_dir)

    # Give instructions
    rospy.loginfo("Press %s button to collect and store waypoint.", collect_button_sym)
    rospy.loginfo("Press %s button to end waypoint collection.", end_button_sym)

    num_waypoints = 0

    # Open file
    with open(coordinates_file_dir, 'w') as coord_file:
        rospy.loginfo("File opened")

        # Main loop
        while not rospy.is_shutdown() and continue_collection:
            rospy.sleep(0.1)  # Introduce a small delay
            time_current = rospy.Time.now()
            if collect_request:
                # Calculate Euclidean distance between consecutive positions
                distance_moved = math.sqrt(
                    (robot_position_x - robot_position_last_x) ** 2 +
                    (robot_position_y - robot_position_last_y) ** 2 +
                    (robot_position_z - robot_position_last_z) ** 2
                )

                if distance_moved > min_move_change:
                    rospy.loginfo("You have moved a significant distance!")
                    rospy.loginfo("Press %s button to collect and store another waypoint.", collect_button_sym)
                    rospy.loginfo("Press %s button to end waypoint collection.", end_button_sym)
                    num_waypoints += 1
                    # Save data or perform actions here
                    coord_file.write("{:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f} {:.8f}\n".format(
                    robot_position_x, robot_position_y, robot_position_z,
                    robot_orientation_x, robot_orientation_y, robot_orientation_z, robot_orientation_w))

                    robot_position_last_x = robot_position_x
                    robot_position_last_y = robot_position_y
                    robot_position_last_z = robot_position_z

                    rospy.loginfo("Waypoint saved to file.")
                else:
                    rospy.logwarn("Waypoint not saved, you have not moved enough")
                    rospy.logwarn("Robot Position (X, Y, Z): (%.4f, %.4f, %.4f)", robot_position_x, robot_position_y, robot_position_z)
                    rospy.logwarn("Last Robot Position (X, Y, Z): (%.4f, %.4f, %.4f)", robot_position_last_x, robot_position_last_y, robot_position_last_z)
            else:
                pass

        rospy.loginfo("End request registered.")

    rospy.loginfo("Closed waypoint file, you have collected %d waypoints.", num_waypoints)
    rospy.loginfo("Ending node...")
    pub_collection_node_ended.publish(Bool(data=True))
    #rospy.shutdown()
    rospy.signal_shutdown("Node shutdown")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
