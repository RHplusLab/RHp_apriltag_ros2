from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'RHp_apriltag_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},                # src/ 디렉토리 하위를 패키지로 인식
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='minipin',
    maintainer_email='ppp6131@yonsei.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_realsense = RHp_apriltag_ros2.apriltag_realsense:main',
            'apriltag_webcam = RHp_apriltag_ros2.apriltag_webcam:main'
        ],
    },
)
