import rospy
import yaml
import os

# Initialize the ROS node
rospy.init_node('path_to_yaml')

# Get the file path and output directory from the ROS parameter server
file_path = rospy.get_param('~file_path')
output_directory = rospy.get_param('~output_directory')

# Extract the file name without the extension
file_name = os.path.splitext(os.path.basename(file_path))[0]

# Read the robot pose data from the file
pose_data = []
with open(file_path, 'r') as file:
    lines = file.readlines()
    for line in lines[1:]:
        pose = list(map(float, line.strip().split()))
        pose_data.append(pose)

# Convert the pose data to the YAML template
yaml_data = {
    'header': {
        'seq': 1,
        'stamp': {'secs': 0, 'nsecs': 0},
        'frame_id': 'map'
    },
    'poses': []
}

for i, pose in enumerate(pose_data):
    yaml_pose = {
        'header': {
            'seq': i,
            'stamp': {'secs': int(pose[0]), 'nsecs': int((pose[0] % 1) * 1e9)},
            'frame_id': 'map'
        },
        'pose': {
            'position': {'x': pose[1], 'y': pose[2], 'z': pose[3]},
            'orientation': {'x': pose[4], 'y': pose[5], 'z': pose[6], 'w': pose[7]}
        }
    }
    yaml_data['poses'].append(yaml_pose)

# Generate the output file path based on the output directory and input file name
output_file_path = os.path.join(output_directory, file_name + '.yaml')

# Write the YAML data to the output file
with open(output_file_path, 'w') as file:
    yaml.dump(yaml_data, file, default_flow_style=False)

rospy.loginfo("Conversion completed. YAML file generated: %s", output_file_path)
