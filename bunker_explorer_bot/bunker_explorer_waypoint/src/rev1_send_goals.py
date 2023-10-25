#!/usr/bin/env python

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import rospkg
from os.path import join

def send_goal(client, x, y, orientation_z, orientation_w):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.z = orientation_z
    goal.target_pose.pose.orientation.w = orientation_w

    client.send_goal(goal)
    client.wait_for_result()

def main():
    rospy.init_node('move_base_action_client')

    client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
    client.wait_for_server()

    rospack = rospkg.RosPack()
    package_path = rospack.get_path('bunker_explorer_navigation')
    file_path = join(package_path, 'waypoint_files', 'points_slam_indoor.txt')

    goals = []
    with open(file_path, 'r') as file:
        lines = file.readlines()
        for line in lines:
            data = line.strip().split()
            if len(data) >= 7:
                x, y, _, _, _, oz, ow = map(float, data)
                goals.append({'x': x, 'y': y, 'oz': oz, 'ow': ow})

    try:
        while not rospy.is_shutdown():
            for goal in goals:
                send_goal(client, goal['x'], goal['y'], goal['oz'], goal['ow'])
                rospy.sleep(2)  # Sleep between goals
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()
