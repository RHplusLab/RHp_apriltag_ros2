# apriltag_webcam.launch.py

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Setting relative directory
    package_dir = get_package_share_directory('rhp_apriltag_ros2')
    rviz_config_path = os.path.join(package_dir, 'config', 'apriltag_rviz.rviz')

    return LaunchDescription([
        # Execute AprilTag webcam node
        Node(
            package='rhp_apriltag_ros2',
            executable='apriltag_webcam',
            name='apriltag_webcam_node',
            output='screen'
        ),

        # Run rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
