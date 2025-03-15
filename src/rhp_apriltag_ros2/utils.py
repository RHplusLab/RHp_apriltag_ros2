import numpy as np
import cv2

class CameraInfo:
    def __init__(self, fx, ppx, fy, ppy, width, height):
        self.fx = fx
        self.ppx = ppx
        self.fy = fy
        self.ppy = ppy
        self.width = width
        self.height = height
        
def draw_cube(overlay, camera_params, tag_size, pose, z_sign=1):
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
    
    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)

def rotation_matrix_to_quaternion(R):
    """ 회전 행렬을 사원수로 변환 """
    qw = np.sqrt(1.0 + R[0, 0] + R[1, 1] + R[2, 2]) / 2
    qx = (R[2, 1] - R[1, 2]) / (4 * qw)
    qy = (R[0, 2] - R[2, 0]) / (4 * qw)
    qz = (R[1, 0] - R[0, 1]) / (4 * qw)
    return qx, qy, qz, qw


