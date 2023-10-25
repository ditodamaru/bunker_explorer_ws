#!/usr/bin/env python

import os
import rospkg
import rospy
import xml.etree.ElementTree as ET

def find_rtabmap_databases(directory):
    database_files = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.db'):
                database_files.append(os.path.join(root, file))
    return database_files

def update_launch_file(launch_file_path, new_map_filename):
    tree = ET.parse(launch_file_path)
    root = tree.getroot()

    for node in root.findall(".//node[@name='map_saver_node']"):
        args = node.get('args')
        if args and args.startswith('-f $(find bunker_explorer_navigation)/maps/'):
            new_args = args.replace(args.split('/')[-1], new_map_filename)
            node.set('args', new_args)

    tree.write(launch_file_path)

def main():
    try:
        # Search for RTAB-Map databases in ~/.ros/
        home_dir = os.path.expanduser('~')
        ros_dir = os.path.join(home_dir, '.ros')
        rtabmap_databases = find_rtabmap_databases(ros_dir)

        if len(rtabmap_databases) == 0:
            print("No RTAB-Map databases found.")
        else:
            print("Found RTAB-Map databases:")
            for idx, db_path in enumerate(rtabmap_databases):
                print("{}: {}".format(idx + 1, db_path))

            selected_idx = int(input("Select a database (enter the index): ")) - 1
            selected_db = rtabmap_databases[selected_idx]
            map_filename = os.path.splitext(os.path.basename(selected_db))[0]

            # Update the launch file
            launch_file_path = rospkg.RosPack().get_path('bunker_explorer_navigation') + '/launch/rev1_map_saver.launch'
            update_launch_file(launch_file_path, map_filename)
            print("Launch file updated with map filename: {}".format(map_filename))
    except KeyboardInterrupt:
        print("\nScript interrupted. Exiting gracefully.")

if __name__ == '__main__':
    main()
