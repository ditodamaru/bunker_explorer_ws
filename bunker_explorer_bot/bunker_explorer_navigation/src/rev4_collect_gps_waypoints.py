#!/usr/bin/env python

import rospy
import rospkg
import math
import sys
import std_msgs.msg
from sensor_msgs.msg import NavSatFix, Joy
from std_msgs.msg import Bool, Char

collect_request = False
continue_collection = True
lati_point = 0
longi_point = 0
lati_last = 0
longi_last = 0
min_coord_change = 10 * math.pow(10, -6)
end_button_sym = ""
collect_button_sym = ""
end_button_num = ""
collect_button_num = ""

def joy_CB(joy_msg):
    global collect_request, continue_collection
    if joy_msg.buttons[0] == ord(collect_button_num):
        collect_request = True
    else:
        collect_request = False

    if joy_msg.buttons[0] == ord(end_button_num):
        continue_collection = False

def keyboard_CB(msg):
    global collect_request, continue_collection
    key_pressed = msg.data
    #key_pressed = ord(msg.data)  # Convert received char to ASCII value
    print("Received key:", key_pressed)  # Add this line


    if key_pressed == 115: # ASCII value for 's'
        collect_request = True
    elif key_pressed == 101: # ASCII value for 'e'
        continue_collection = False

def filtered_gps_CB(gps_msg):
    global lati_point, longi_point
    lati_point = gps_msg.latitude
    longi_point = gps_msg.longitude

def main():
    global collect_request, continue_collection, lati_point, longi_point, lati_last, longi_last, min_coord_change
    global end_button_sym, collect_button_sym, end_button_num, collect_button_num

    # Initialize node
    rospy.init_node("collect_gps_waypoints")

    # Initialize subscribers
    rospy.Subscriber("/keyboard_input", Char, keyboard_CB)
    rospy.Subscriber("/joy_teleop/joy", Joy, joy_CB)
    rospy.Subscriber("/ublox_gps/fix", NavSatFix, filtered_gps_CB)
    rospy.loginfo("Initiated collect_gps_waypoints node")

    # Initialize publisher
    pub_collection_node_ended = rospy.Publisher("/outdoor_waypoint_nav/collection_status", Bool, queue_size=100)

    # Read parameters
        #coordinates_file_dir = rospy.get_param("/outdoor_waypoint_nav/coordinates_file")
    #coordinates_file_dir = rospy.get_param("coordinates_file_dir")
    coordinates_file_dir = "/home/andi/catkin_devel_ws/src/bunker_explorer_bot/bunker_explorer_navigation/waypoint_files/points_outdoor.txt"
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
            #rospy.spin()
            rospy.sleep(0.1)  # Introduce a small delay
            time_current = rospy.Time.now()
            if collect_request:
                difference_lat = abs((lati_point - lati_last) * math.pow(10, 6)) * math.pow(10, -6)
                difference_long = abs((longi_point - longi_last) * math.pow(10, 6)) * math.pow(10, -6)

                if difference_lat > min_coord_change or difference_long > min_coord_change:
                    rospy.loginfo("You have collected another waypoint!")
                    rospy.loginfo("Press %s button to collect and store another waypoint.", collect_button_sym)
                    rospy.loginfo("Press %s button to end waypoint collection.", end_button_sym)
                    num_waypoints += 1
                    coord_file.write("{:.8f} {:.8f}\n".format(lati_point, longi_point))
                    lati_last = lati_point
                    longi_last = longi_point
                else:
                    rospy.logwarn("Waypoint not saved, you have not moved enough")
                    rospy.logwarn("New Latitude: %.8f   Last Latitude: %.8f", lati_point, lati_last)
                    rospy.logwarn("New Longitude: %.8f   Last Longitude: %.8f", longi_point, longi_last)
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
