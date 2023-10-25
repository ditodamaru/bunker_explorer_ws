#! /usr/bin/env python
import rospy
import time
import actionlib
import rospkg  # Import the rospkg module
from os.path import join

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback

def feedback_callback(feedback):
    print('[Feedback] Going to Goal Pose...')

rospy.init_node('move_base_action_client')
client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
client.wait_for_server()

# Initialize the rospkg object
rospack = rospkg.RosPack()

# Construct the full file path using rospack
package_path = rospack.get_path('bunker_explorer_waypoint')
file_path = join(package_path, 'waypoint_files', 'waypoint_20230901_0235.txt')

# Read the file and split it into lines
with open(file_path, 'r') as file:
    lines = file.readlines()

# Iterate through the lines to create and send goals
for line in lines:
    data = line.strip().split()
    if len(data) >= 7:  # Make sure there are enough values in the line
        x, y, z, qx, qy, qz, qw = map(float, data)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        client.send_goal(goal, feedback_cb=feedback_callback)
        client.wait_for_result()
        print('[Result] State:', client.get_state())
