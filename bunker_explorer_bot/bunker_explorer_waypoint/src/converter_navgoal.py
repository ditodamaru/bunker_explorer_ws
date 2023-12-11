#!/usr/bin/env python

import os
import yaml
import rospkg

# Initialize ROS package manager
rospack = rospkg.RosPack()

# Get the path to the 'bunker_explorer_waypoint' package
package_path = rospack.get_path('bunker_explorer_waypoint')

# Define the directory where waypoint files are located
waypoint_dir = os.path.join(package_path, 'waypoint_files')

# List all files in the waypoint directory
waypoint_files = os.listdir(waypoint_dir)

if not waypoint_files:
    print("No waypoint files found in the directory.")
else:
    print("Available waypoint files:")
    for i, filename in enumerate(waypoint_files):
        print("{}. {}".format(i + 1, filename))

    # Ask the user to select a file for conversion
    while True:
        try:
            selection = int(input("Select a waypoint file (1-{}): ".format(len(waypoint_files))))
            if 1 <= selection <= len(waypoint_files):
                selected_file = waypoint_files[selection - 1]
                break
            else:
                print("Invalid selection. Please enter a valid number.")
        except ValueError:
            print("Invalid input. Please enter a number.")

    # Define the input and output file paths
    input_file = os.path.join(waypoint_dir, selected_file)
    #output_file = "/home/andi/catkin_redevel_ws/src/tracking_pid/trajectories/waypoint_20230829_1828.yaml"
    #output_file = "/home/$USER/github/bunker_eplorer_ws/src/bunker_explorer_ws/tracking_pid/trajectories/waypoint_conv.yaml"
    #/home/parlab/github/bunker_eplorer_ws/src/bunker_explorer_ws/tracking_pid/trajectories
    output_file = "/home/parlab/github/bunker_eplorer_ws/src/bunker_explorer_ws/tracking_pid/trajectories/waypoint_conv.yaml"

    # Define the frame_id without single quotes
    frame_id = 'map'

    # Read the selected input waypoint file
    with open(input_file, 'r') as file:
        lines = file.readlines()

    poses = []

    # Process each line in the selected input file
    for seq, line in enumerate(lines):
        values = line.split()
        pose_data = {
            "header": {
                "seq": seq + 1,  # Incremented seq to start from 1
                "stamp": {
                    "secs": 0,
                    "nsecs": 0
                },
                "frame_id": frame_id
            },
            "pose": {
                "position": {
                    "x": float(values[0]),
                    "y": float(values[1]),
                    "z": float(values[2])
                },
                "orientation": {
                    "x": float(values[3]),
                    "y": float(values[4]),
                    "z": float(values[5]),
                    "w": float(values[6])
                }
            }
        }
        poses.append(pose_data)

    # Write the YAML data to the output file
    output_data = {
        "header": {
            "seq": 1,
            "stamp": {
                "secs": 0,
                "nsecs": 0
            },
            "frame_id": frame_id
        },
        "poses": poses
    }

    with open(output_file, 'w') as file:
        yaml.dump(output_data, file, default_flow_style=False)

    print("Waypoint data from {} has been converted and saved to {}".format(selected_file, output_file))
