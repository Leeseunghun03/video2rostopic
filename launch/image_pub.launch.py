from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_pub',
            executable='image_pub_node',
            name='image_pub',
            output='screen',
            parameters=[
                {'camera_topic': '/camera/image_raw'},
                {'camera_info_topic': '/camera/camera_info'},
                # {'camera_info_url': 'file:///path/to/camera_calibration.yaml'},
                {'img_path': '/home/sh/Downloads/soccer.mp4'},
                {'frame_id': 'camera'},
                {'pub_rate': 30.0},
                {'start_sec': 0},
                {'repeat': True}
            ]
        )
    ])
