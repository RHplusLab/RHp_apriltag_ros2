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
from .utils import rotation_matrix_to_quaternion, _draw_cube

# depth 영상용 카메라 정보 클래스
class CameraInfo:
    def __init__(self, fx, ppx, fy, ppy, width, height):
        self.fx = fx
        self.ppx = ppx
        self.fy = fy
        self.ppy = ppy
        self.width = width
        self.height = height

class AprilTagPublisher(Node):
    def __init__(self):
        super().__init__('apriltag_publisher')
        self.pose_publisher = self.create_publisher(PoseStamped, '/apriltag_pose', 10)
        self.image_publisher = self.create_publisher(Image, '/apriltag_image', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.detect_apriltag)

        # 카메라 설정
        self.cam_params_rgb = (638.956, 638.394, 630.634, 367.101)

        # AprilTag 검출기 초기화
        options = apriltag.DetectorOptions(families="tag25h9")
        self.detector = apriltag.Detector(options)
        self.tag_size = 0.04

        # RealSense 설정
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.get_logger().info("RealSense 카메라 스트림 시작...")

    def detect_apriltag(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        color_image = np.asanyarray(color_frame.get_data())
        img_gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        results, _ = self.detector.detect(img_gray, return_image=True)
        overlay = copy.deepcopy(color_image)

        for r in results:
            pose, _, _ = self.detector.detection_pose(r, self.cam_params_rgb, self.tag_size)
            self._draw_cube(overlay, self.cam_params_rgb, self.tag_size, pose)
            
            # 위치 정보 변환
            position = pose[:3, 3]
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            msg.pose.position.x = position[0]
            msg.pose.position.y = position[1]
            msg.pose.position.z = position[2]
            
            # 회전 정보 (행렬 -> 사원수 변환 필요)
            rotation_matrix = pose[:3, :3]
            qx, qy, qz, qw = self.rotation_matrix_to_quaternion(rotation_matrix)
            msg.pose.orientation.x = qx
            msg.pose.orientation.y = qy
            msg.pose.orientation.z = qz
            msg.pose.orientation.w = qw
            
            self.pose_publisher.publish(msg)
            # self.get_logger().info(f'Published AprilTag Pose: x={position[0]:.3f}, y={position[1]:.3f}, z={position[2]:.3f}')

        # 검출된 태그가 포함된 이미지를 퍼블리시
        img_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='bgr8')
        self.image_publisher.publish(img_msg)
    
    def shutdown(self):
        self.pipeline.stop()
        self.get_logger().info("카메라 스트림 종료.")


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
