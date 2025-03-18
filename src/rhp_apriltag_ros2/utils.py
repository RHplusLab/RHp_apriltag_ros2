import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

class CameraInfo:
    def __init__(self, fx, ppx, fy, ppy, width, height):
        self.fx = fx
        self.ppx = ppx
        self.fy = fy
        self.ppy = ppy
        self.width = width
        self.height = height
        
def draw_cube(overlay, camera_params, tag_size, pose, tag_id, z_sign=1):
    """ 검출된 태그 위에 cube를 그린다. """
    # 객체 좌표 (8개 점)
    opoints = np.array([
        -1, -1, 0,          # 아래 네 점
        1, -1, 0,
        1,  1, 0,
        -1,  1, 0,
        -1, -1, -2*z_sign,  # 위 네 점
        1, -1, -2*z_sign,
        1,  1, -2*z_sign,
        -1,  1, -2*z_sign,
    ]).reshape(-1, 1, 3) * 0.5 * tag_size

    # 점들을 잇는 선(edge) 인덱스
    edges = np.array([
        0, 1,
        1, 2,
        2, 3,
        3, 0,
        0, 4,
        1, 5,
        2, 6,
        3, 7,
        4, 5,
        5, 6, 
        6, 7,
        7, 4
    ]).reshape(-1, 2)
    
    fx, fy, cx, cy = camera_params
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]
    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, np.zeros(5))
    ipoints = [tuple(pt) for pt in np.round(ipoints).astype(int).reshape(-1, 2)]
    
    color_tables = [
        (255, 0, 0),    # red
        (0, 255, 0),    # green
        (0, 0, 255),    # blue
        (255, 255, 0),  # yellow
        (0, 255, 255),  # cyan
        (255, 0, 255),  # magenta
        (255, 165, 0),  # orange
        (128, 0, 128),  # purple
        (255, 192, 203),# pink
        (165, 42, 42)   # brown
    ]

    if tag_id >= len(color_tables) or tag_id < 0:
        print(f"Invalid tag_id: {tag_id}, skipping draw_cube()")
        return 

    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], color_tables[tag_id], 1, 16)

def rotation_matrix_to_quaternion(R):
    """ Rotation Matrix -> Quaternion"""
    qw = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    return qx, qy, qz, qw

def construct_transform(T_co, surface_offset=(0.0,0.0)):
    """ Change Coordination Tbo = Tbc * Tco (b:base, c:camera, o:object)   """
    
    # camera position based on robot base (Unit: m)
    camera_offset = np.array([0.02473, 0.075, 0.4092])

    angle_rad = np.arccos(0.8) # angle: (camera-z axis <-> robot base xyplane)
    axis = np.array([1,0,0]) # based : x-axis 
    R_bc = R.from_rotvec(angle_rad * axis).as_matrix() # rotation matrix

    # robot_base -> camera (Tbc)
    T_bc = np.eye(4)
    T_bc[:3, :3] = R_bc
    T_bc[:3, 3] = camera_offset

    T_bo = T_bc @ T_co 

    # if robot base move
    T_surface = np.eye(4)
    T_surface[:3, 3] = np.array([surface_offset[0], surface_offset[1], 0.0]) # consider (x,y) moving

    T_bo = T_surface @ T_bo

    return T_bo