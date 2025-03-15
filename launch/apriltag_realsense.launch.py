# apriltag_realsense.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 상대 경로 설정 (패키지 내 config 디렉토리를 사용)
    package_dir = get_package_share_directory('rhp_apriltag_ros2')
    rviz_config_path = os.path.join(package_dir, 'config', 'apriltag_rviz.rviz')

    return LaunchDescription([
        # AprilTag 검출 노드 실행
        Node(
            package='rhp_apriltag_ros2',
            executable='apriltag_realsense',
            name='apriltag_realsense_node',
            output='screen'
        ),
        # RViz2 실행 (상대 경로 사용)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        )
    ])
