#!/usr/bin/env python

import os
import yaml
import rospkg

import errno  # Import the errno module

# Define the path to the waypoint files directory
waypoint_dir = '/home/andi/catkin_redevel_ws/src/bunker_explorer_bot/bunker_explorer_waypoint/waypoint_files/'

# List all files in the directory
waypoint_files = os.listdir(waypoint_dir)

# Display the list of available waypoint files and let the user select one
print("Available waypoint files:")
for i, file_name in enumerate(waypoint_files):
    print(str(i + 1) + ". " + file_name)

# Ask the user to select a waypoint file
selected_file_index = int(input("Select a waypoint file (enter the file number): ")) - 1

# Check if the selected index is valid
if 0 <= selected_file_index < len(waypoint_files):
    selected_file_name = waypoint_files[selected_file_index]
    input_file_path = os.path.join(waypoint_dir, selected_file_name)
    
    # Modify the output filename to include "_filtered.txt" suffix
    output_file_name = selected_file_name.replace('.txt', '_filtered.txt')
    output_file_path = os.path.join(
        '/home/andi/catkin_redevel_ws/src/bunker_explorer_bot/bunker_explorer_waypoint/waypoint_filtered/',
        output_file_name
    )

    # Attempt to create the output directory, handle exceptions
    try:
        os.makedirs(os.path.dirname(output_file_path))
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise

    # Read the input file and store unique waypoints
    unique_waypoints = set()

    with open(input_file_path, 'r') as input_file:
        for line in input_file:
            waypoint = line.strip()
            if waypoint not in unique_waypoints:
                unique_waypoints.add(waypoint)

    # Write the unique waypoints to the output file
    with open(output_file_path, 'w') as output_file:
        output_file.write('\n'.join(unique_waypoints))

    # Print the number of waypoints before and after filtering
    num_waypoints_before = len(unique_waypoints)
    num_waypoints_after = len(unique_waypoints)

    print("Number of waypoints before filtering: " + str(num_waypoints_before))
    print("Number of waypoints after filtering: " + str(num_waypoints_after))
    print("Filtered waypoints saved to: " + output_file_path)
else:
    print("Invalid file selection. Please enter a valid file number.")