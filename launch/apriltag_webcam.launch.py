# apriltag_webcam.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='RHp_apriltag_ros2',
            executable='apriltag_webcam',
            name='apriltag_webcam_node'
        )
    ])
