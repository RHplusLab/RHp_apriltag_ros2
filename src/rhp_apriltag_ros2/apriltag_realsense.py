#!/usr/bin/env python3
import argparse
import numpy as np
import cv2
import apriltag
import copy
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .utils import *

from rhp_apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Quaternion

class AprilTagPublisher(Node):
    def __init__(self, surface_offset):
        super().__init__('apriltag_publisher')
        self.surface_offset = surface_offset

        self.apriltag_pub = self.create_publisher(AprilTagDetectionArray, '/apriltag_detections', 10)
        self.image_pub = self.create_publisher(Image, '/apriltag_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0/30, self.detect_apriltag)
    
        # Camera Hyperparameter
        self.cam_params_rgb = (638.956, 638.394, 630.634, 367.101)

        # Initialize AprilTag Detector
        options = apriltag.DetectorOptions(families="tag25h9")
        self.detector = apriltag.Detector(options)
        self.tag_size = 0.04

        # RealSense Setting
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.get_logger().info("Streamling RealSense...")

    def detect_apriltag(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        img_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        results, _ = self.detector.detect(img_gray, return_image=True)
        overlay = copy.deepcopy(color_image)

        # Make Message for Detection Tags
        now_ = self.get_clock().now().to_msg()
        detection_array_msg = AprilTagDetectionArray()
        detection_array_msg.header.stamp = now_
        detection_array_msg.header.frame_id = "camera_link"

        for r in results:
            # calculate pose and visualization
            pose, _, _ = self.detector.detection_pose(r, self.cam_params_rgb, self.tag_size)
            draw_cube(overlay, self.cam_params_rgb, self.tag_size, pose, r.tag_id)
            
            # Make the AprilTag Message
            detection_msg = AprilTagDetection()
            detection_msg.family =  str(r.tag_family, 'utf-8')
            detection_msg.id = r.tag_id

            detection_msg.center.x = float(r.center[0])
            detection_msg.center.y = float(r.center[1])
            detection_msg.center.z = 0.0

            for i in range(4):
                corner_x, corner_y = r.corners[i][0], r.corners[i][1]
                detection_msg.corners[i].x = corner_x
                detection_msg.corners[i].y = corner_y
                detection_msg.corners[i].z = 0.0
            
            robot_base_pose = construct_transform(pose)
            position = robot_base_pose[:3, 3]
            rotation_matrix = robot_base_pose[:3, :3]
            qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_matrix)
            
            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = now_
            pose_msg.header.frame_id = "robot_base"

            pose_msg.pose.pose.position.x = float(position[0])
            pose_msg.pose.pose.position.y = float(position[1])
            pose_msg.pose.pose.position.z = float(position[2])

            pose_msg.pose.pose.orientation.x = float(qx)
            pose_msg.pose.pose.orientation.y = float(qy)
            pose_msg.pose.pose.orientation.z = float(qz)
            pose_msg.pose.pose.orientation.w = float(qw)

            pose_msg.pose.covariance = [0.0] * 36

            detection_msg.pose = pose_msg

            detection_array_msg.detections.append(detection_msg)

        self.apriltag_pub.publish(detection_array_msg)
        
        # Visualize
        img_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        self.image_pub.publish(img_msg)
    
    def shutdown(self):
        self.pipeline.stop()
        self.get_logger().info("End Stream..")

def parse_surface_oofset(offset_str):

    try:
        cleaned_str = offset_str.replace("(","").replace(")","").replace(" ","")
        x_str, y_str = cleaned_str.split(",")
        return float(x_str), float(y_str)
    except Exception as e:
        raise ValueError(f"Invalid format for surface_offset: {offset_str}. Expected format: 'x,y' or '(x,y)") from e

def main():
    parser = argparse.ArgumentParser(description="AprilTag Detection by RealSense")
    parser.add_argument('--surface_offset', type=str, default="0.0,0.0", 
                        help="Real robot base offset in format 'x,y' (e.g., '0.5,0.4' or '(0.8, 0.9)')")
    
    args = parser.parse_args()
    try:
        surface_offset = parse_surface_offset(args.surface_offset)
    except ValueError as e:
        print(e)
        return
    
    rclpy.init()
    node = AprilTagPublisher(surface_offset=surface_offset)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
