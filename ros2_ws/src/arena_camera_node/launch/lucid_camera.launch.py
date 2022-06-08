import launch
from launch_ros.actions import Node

def generate_launch_description():
    throttle_remap = Node(
        package="arena_camera_node",
        executable="camera_node",
        parameters=[
            {
                # 'serial': ''
                'pixelformat': 'rgb8',
                # 'exposure_time': 80000.0, # microseconds
                # 'frame_rate': 10.0,
                # 'test_pattern': 'Pattern3',
                'trigger_mode': False,
                "ptp": True,
                'frame_id': "/camera",
                'qos_history': 'keep_last',
                'qos_history_depth': 1,
                'qos_reliability': 'best_effort'
            }
        ]
    )

    return launch.LaunchDescription([throttle_remap])
