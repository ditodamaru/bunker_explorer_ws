import yaml

# Read the robot pose data from the text file
pose_data = []
with open('poses_warehouse_rgbd.txt', 'r') as file:
    for line in file:
        pose_data.append(list(map(float, line.strip().split())))

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

# Write the YAML data to a file
with open('poses_warehouse_rgbd.yaml', 'w') as file:
    yaml.dump(yaml_data, file, default_flow_style=False)

print("Conversion completed. YAML file generated.")
