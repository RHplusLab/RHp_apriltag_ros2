import numpy as np
import cv2
import apriltag
import copy
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from .utils import rotation_matrix_to_quaternion, draw_cube

from rhp_apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseWithCovariance, Pose, Quaternion

class AprilTagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')
        self.apriltag_pub = self.create_publisher(AprilTagDetectionArray, '/apriltag_detections', 10)
        self.image_pub = self.create_publisher(Image, '/apriltag_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0/30, self.detect_apriltag)

        # Camera Hyperparameter 
        self.cam_params_rgb = (600.0, 600.0, 600.0, 300.0) 

        # Initialize AprilTag Detector
        options = apriltag.DetectorOptions(families="tag25h9")
        self.detector = apriltag.Detector(options)
        self.tag_size = 0.04

        # Webcam Setting 
        self.cap = cv2.VideoCapture(0)  
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)  
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)   
        self.cap.set(cv2.CAP_PROP_FPS, 30)            
        self.get_logger().info("Streaming Webcam...")

    def detect_apriltag(self):
    
        # Read a Frame 
        ret, frame = self.cap.read()  
        if not ret:
            return

        img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        results, _ = self.detector.detect(img_gray, return_image=True)
        overlay = copy.deepcopy(frame)

        # Make Message for Detection Tags
        now_ = self.get_clock().now().to_msg()
        detection_array_msg = AprilTagDetectionArray()
        detection_array_msg.header.stamp = now_
        detection_array_msg.header.frame_id = "camera_link"

        
        
        for r in results:
            # Calculate pose and visualization using webcam parameters
            pose, _, _ = self.detector.detection_pose(r, self.cam_params_rgb, self.tag_size)
            draw_cube(overlay, self.cam_params_rgb, self.tag_size, pose,r.tag_id,z_sign=1)

            # Make the AprilTag Message
            detection_msg = AprilTagDetection()
            detection_msg.family = str(r.tag_family, 'utf-8')
            detection_msg.id = r.tag_id

            detection_msg.center.x = float(r.center[0])
            detection_msg.center.y = float(r.center[1])
            detection_msg.center.z = 0.0

            for i in range(4):
                corner_x, corner_y = r.corners[i][0], r.corners[i][1]
                detection_msg.corners[i].x = corner_x
                detection_msg.corners[i].y = corner_y
                detection_msg.corners[i].z = 0.0

            position = pose[:3, 3]
            rotation_matrix = pose[:3, :3]

            qx, qy, qz, qw = rotation_matrix_to_quaternion(rotation_matrix)
            """
            from scipy.spatial.transform import Rotation
            r = Rotation.from_matrix(rotation_matrix)
            quat = r.as_quat()
            qx, qy, qz, qw = quat
            """

            pose_msg = PoseWithCovarianceStamped()
            pose_msg.header.stamp = now_
            pose_msg.header.frame_id = "camera_link"

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
        self.cap.release()  # 웹캠 스트림 종료
        self.get_logger().info("End Stream..")


def main():
    rclpy.init()
    node = AprilTagPublisher()
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
