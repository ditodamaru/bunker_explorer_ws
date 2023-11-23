#!/usr/bin/env python

import os
import yaml
import rospy
from geometry_msgs.msg import PoseStamped
from datetime import datetime

class GoalDataSaver:
    def __init__(self):
        self.extracted_data = []
        self.package_path = rospy.get_param('/bunker_explorer_waypoint')
        self.waypoint_file_path = None  # Initialize the file path
        self.create_file_path()  # Create the initial file path
        rospy.Subscriber('/goal_2d', PoseStamped, self.goal_callback)
        
    def goal_callback(self, msg):
        position = msg.pose.position
        orientation = msg.pose.orientation
        self.extracted_data.append(
            "{:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f} {:.9f}".format(
                position.x, position.y, position.z,
                orientation.x, orientation.y, orientation.z, orientation.w
            )
        )

        self.save_data()  # Save the data
        #self.calculate_waypoint_count()  # Calculate waypoint count after saving
        self.extracted_data = []  # Clear the data after saving



    def create_file_path(self):
        current_time = datetime.now()
        file_name = 'waypoint_' + current_time.strftime('%Y%m%d_%H%M') + '.txt'
        self.waypoint_file_path = os.path.join(self.package_path, file_name)
        rospy.loginfo("Using waypoint file: '%s'", self.waypoint_file_path)

    def save_data(self):
        with open(self.waypoint_file_path, 'a') as file:  # Use 'a' mode for appending
            for line in self.extracted_data:
                file.write(line + '\n')
        rospy.loginfo("Data appended to '%s'", self.waypoint_file_path)
        self.calculate_waypoint_count()  # Calculate waypoint count based on the existing file

    # def calculate_waypoint_count(self):
    #     waypoint_count = len(self.extracted_data)
    #     rospy.loginfo("Total waypoints collected: %d", waypoint_count)

    def calculate_waypoint_count(self):
        waypoint_count = 0
        if self.waypoint_file_path is not None:
            with open(self.waypoint_file_path, 'r') as file:
                waypoint_count = sum(1 for line in file)
        rospy.loginfo("Total waypoints collected: %d", waypoint_count) 

def main():
    rospy.init_node('save_goal_data_node')
    GoalDataSaver()
    rospy.spin()

if __name__ == "__main__":
    main()