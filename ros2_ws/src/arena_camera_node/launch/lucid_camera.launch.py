import launch
from launch_ros.actions import Node

def generate_launch_description():
    camera_node = Node(
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
                'ptp': True,
                'camera_frame_id': 'lucid_camera',
                'camera_info_path': '',
                'qos_history': 'keep_last',
                'qos_history_depth': 1,
                'qos_reliability': 'best_effort'
            }],
        # remappings=[
        #     ('image_raw', '/lucid_camera/image_raw'),
        #     ('trigger_capture', '/lucid_camera/trigger_capture'),
        #     ('camera_info', '/lucid_camera/camera_info')],
        namespace="/lucid_camera"
    )

    return launch.LaunchDescription([camera_node])
