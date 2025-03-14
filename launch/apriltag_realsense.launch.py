# apriltag_realsense.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='RHp_apriltag_ros2',         # 패키지 이름 (package.xml에 설정한 이름)
            executable='apriltag_realsense', # setup.py (entry point)에서 등록한 실행 파일 이름
            name='apriltag_realsense_node'
        )
    ])
