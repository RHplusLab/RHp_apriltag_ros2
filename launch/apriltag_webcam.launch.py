# apriltag_webcam.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rhp_apriltag_ros2',
            executable='apriltag_webcam',
            name='apriltag_webcam_node'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )


    ])
