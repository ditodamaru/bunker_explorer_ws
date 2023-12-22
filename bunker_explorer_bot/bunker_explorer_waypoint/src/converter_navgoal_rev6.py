#!/usr/bin/env python
import yaml

def convert_txt_to_yaml(input_file, output_file):
    with open(input_file, 'r') as f:
        lines = f.readlines()

    waypoints = []
    for i, line in enumerate(lines[1:]):  # Skip the header line
        values = map(float, line.strip().split())
        waypoint = {
            'header': {
                'seq': 1,
                'stamp': {
                    'secs': 0,
                    'nsecs': 0
                },
                'frame_id': "'map'"
            },
            'poses': [
                {
                    'header': {
                        'seq': 0,
                        'stamp': {
                            'secs': 0,
                            'nsecs': 0
                        },
                        'frame_id': "'map'"
                    },
                    'pose': {
                        'position': {
                            'x': values[0],
                            'y': values[1],
                            'z': values[2]
                        },
                        'orientation': {
                            'x': 0.000000000,
                            'y': 0.000000000,
                            'z': 0.000000000,
                            'w': values[6]
                        }
                    }
                }
            ]
        }
        waypoints.append(waypoint)

    yaml_data = yaml.dump(waypoints, default_flow_style=False)

    with open(output_file, 'w') as f:
        f.write(yaml_data)


if __name__ == "__main__":
    #input_file = "waypoints.txt"
    input_file = "/home/andi/catkin_redevel_ws/src/bunker_explorer_bot/bunker_explorer_waypoint/waypoint_files/waypoint_20231120_1216.txt"
    #output_file = "waypoints.yaml"
    output_file = "/home/andi/catkin_redevel_ws/src/tracking_pid/trajectories/path.yaml"
    convert_txt_to_yaml(input_file, output_file)
