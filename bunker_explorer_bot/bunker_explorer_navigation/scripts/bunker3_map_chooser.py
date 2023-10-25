#!/usr/bin/env python

import os
import rospkg
import rospy
import xml.etree.ElementTree as ET

def find_yaml_files(directory):
    yaml_files = []
    for file in os.listdir(directory):
        if file.endswith('.yaml'):
            yaml_files.append(file)
    return yaml_files

def update_launch_file(launch_file_path, new_map_filename):
    tree = ET.parse(launch_file_path)
    root = tree.getroot()

    for node in root.findall(".//node[@name='bunker3_map_chooser_node']"):
        node.set('args', "$(arg map_dir)/{}".format(new_map_filename))

    tree.write(launch_file_path)

def main():
    try:
        # Directory containing .yaml files
        map_dir = os.path.join(rospkg.RosPack().get_path('bunker_explorer_navigation'), 'maps')
        
        # List .yaml files
        yaml_files = find_yaml_files(map_dir)

        if not yaml_files:
            print("No .yaml files found in the directory.")
        else:
            print("Found .yaml files:")
            for idx, yaml_file in enumerate(yaml_files):
                print("{}: {}".format(idx + 1, yaml_file))

            selected_idx = int(input("Select a .yaml file (enter the index): ")) - 1
            selected_yaml = yaml_files[selected_idx]

            # Update the launch file
            launch_file_path = rospkg.RosPack().get_path('bunker_explorer_navigation') + '/launch/bunker3_navigation.launch'
            update_launch_file(launch_file_path, selected_yaml)
            print("Launch file updated with .yaml file: {}".format(selected_yaml))
    except KeyboardInterrupt:
        print("\nScript interrupted. Exiting gracefully.")

if __name__ == '__main__':
    main()
