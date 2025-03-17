# apriltag_realsense.launch.py

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 상대 경로 설정 (패키지 내 config 디렉토리를 사용)
    package_dir = get_package_share_directory('rhp_apriltag_ros2')
    rviz_config_path = os.path.join(package_dir, 'config', 'apriltag_rviz.rviz')

    surface_offset_arg = DeclareLaunchArgument(
        'surface_offset', default_value='0.0,0.0',
        description='Surface offset in format "x,y" (e.g., "0.5,0.4")'
    )

    return LaunchDescription([

        surface_offset_arg,
        # AprilTag 검출 노드 실행
        Node(
            package='rhp_apriltag_ros2',
            executable='apriltag_realsense',
            name='apriltag_realsense_node',
            output='screen',
            parameters=[{"surface_offset": LaunchConfiguration('surface_offset')}]
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
