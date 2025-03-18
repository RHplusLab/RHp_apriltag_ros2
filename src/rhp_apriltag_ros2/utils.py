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

def calibrate_rigid_3d(pts_src, pts_tgt):
    """
    Computes a 4x4 rigid transformation matrix (rotation + translation)
    that best aligns 'pts_src' to 'pts_tgt' in 3D space using the Kabsch algorithm.

    Args:
        pts_src (np.ndarray): Source points of shape (N, 3).
        pts_tgt (np.ndarray): Target points of shape (N, 3).

    Returns:
        np.ndarray: A 4x4 homogeneous transformation matrix T
                    such that pts_src are best aligned to pts_tgt.
                    (i.e., applying T to pts_src will approximate pts_tgt)
    """
    # Ensure both sets of points have the same shape
    assert pts_src.shape == pts_tgt.shape, "For calibration, Source and target point sets must have the same shape!"

    # 1) Compute centroids of both sets
    c_src = np.mean(pts_src, axis=0)
    c_tgt = np.mean(pts_tgt, axis=0)

    # 2) Shift both sets so that their centroids coincide with the origin
    src_shifted = pts_src - c_src
    tgt_shifted = pts_tgt - c_tgt

    # 3) Compute the cross-covariance matrix H = src_shifted^T * tgt_shifted
    H = src_shifted.T @ tgt_shifted

    # 4) Perform Singular Value Decomposition (SVD) on H
    U, S, Vt = np.linalg.svd(H)
    R_kabsch = Vt.T @ U.T

    # If the determinant of R_kabsch is negative, it means we have a reflection.
    # We fix this by flipping the sign of the last row of Vt.
    if np.linalg.det(R_kabsch) < 0:
        Vt[-1, :] *= -1
        R_kabsch = Vt.T @ U.T

    # 5) Compute the translation vector t
    #    We want: R_kabsch * c_src + t = c_tgt
    #    => t = c_tgt - R_kabsch @ c_src
    t = c_tgt - R_kabsch @ c_src

    # 6) Construct the final 4x4 transformation matrix
    T_calib = np.eye(4)
    T_calib[:3, :3] = R_kabsch
    T_calib[:3, 3] = t

    return T_calib


def construct_transform(T_co, surface_offset=(0.0,0.0)):
    """ Change Coordination Tbo = Tbc * Tco (b:base, c:camera, o:object)   """
    
    # camera position based on robot base (Unit: m)
    camera_offset = np.array([0.02473, 0.075, 0.4092])

    R_flip_xz = np.array([
        [1, 0, 0], # x -> x
        [0, -1, 0], # y -> -y 
        [0, 0, -1], # z -> -z
    ], dtype=float)

    angle_rad = np.arccos(0.8) # angle: (camera-z axis <-> robot base z-axis)
    axis = np.array([0,1,0]) # based : y-axis (pitch rotation) 
    R_tilt = R.from_rotvec(angle_rad * axis).as_matrix() # rotation matrix

    # flip_xz -> tilt 
    R_bc = R_tilt @ R_flip_xz

    # robot_base -> camera (Tbc)
    T_bc = np.eye(4)
    T_bc[:3, :3] = R_bc
    T_bc[:3, 3] = camera_offset

    T_bo = T_bc @ T_co 

    # if robot base move
    T_surface = np.eye(4)
    T_surface[:3, 3] = np.array([surface_offset[0], surface_offset[1], 0.0]) # consider (x,y) moving

    T_bo = T_surface @ T_bo

    # -----------------------------------------------------
    # (8) Calibration step using 4 points:
    #     We want to align "result_points" to "ground_truth_points"
    #     using the Kabsch algorithm.
    # -----------------------------------------------------
    CALIB_SRC = np.array([
        [-0.19,  -0.09,  0.02],   
        [-0.24,  -0.029, -0.06],  
        [-0.15,  -0.03,  -0.12],  
        [-0.318, -0.032,  0.004], 
        [-0.193, 0.0604, -0.17],
        [-0.1746, 0.0858, -0.142],
        [-0.29687,  0.0620, 0.03886],
    ])
    CALIB_TGT = np.array([
        [0.3,   0.0,   0.035],  
        [0.4,   0.0,   0.035],  
        [0.4,  -0.1,   0.035],  
        [0.4,   0.1,   0.035], 
        [0.5,  -0.1,   0.075], 
        [0.5,  -0.1,   0.108],
        [0.4,   0.1,   0.138]
    ])

    T_calib = calibrate_rigid_3d(CALIB_SRC, CALIB_TGT)

    T_bo = T_calib @ T_bo


    return T_bo
