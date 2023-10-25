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
        self.waypoint_file_path = None
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
        self.save_data()
        self.calculate_waypoint_count()  # Calculate waypoint count after saving

    def save_data(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.waypoint_file_path = os.path.join(self.package_path, 'waypoint_' + timestamp + '.txt')
        with open(self.waypoint_file_path, 'w') as file:
            for line in self.extracted_data:
                file.write(line + '\n')
        rospy.loginfo("Data saved to '%s'", self.waypoint_file_path)

    def calculate_waypoint_count(self):
        waypoint_count = len(self.extracted_data)
        rospy.loginfo("Total waypoints collected: %d", waypoint_count)

def main():
    rospy.init_node('save_goal_data_node')
    GoalDataSaver()
    rospy.spin()

if __name__ == "__main__":
    main()