# RHp_apriltag_ros2

RHp AprilTag detection and pose estimation.

<div align="center">
<img src="https://github.com/user-attachments/assets/6bdc9849-b60e-43c8-a0cf-62904ad4a5e3" width="550px" alt="Demo">
</div>

> [!NOTE]  
> Check out the demo on YouTube: [Watch Here](https://www.youtube.com/watch?v=BMONxk_isWE)

## AprilTag Overview

AprilTag is a visual fiducial system widely used for pose estimation and camera calibration. It consists of square black-and-white patterns (similar to 2D barcodes) that can be detected and identified in images. The system provides robust detection and computes the precise 3D position and orientation (pose) of each tag relative to the camera.

### Detection Output
<div align="center">
  <img src="https://github.com/user-attachments/assets/dfb8a2c8-abde-499b-bb9b-655b47983003" width="250px" alt="AprilTag Detection Illustration">
</div>
As shown in the illustration above, AprilTag detections are returned as an array, where each entry corresponds to a detected tag in the input image. For each detection, the following information is provided:
- Tag ID: A unique identifier for the AprilTag (encoded as a 2D barcode).
- Corners: The four corner coordinates of the tag in the image: `(x0, y0)`, `(x1, y1)`, `(x2, y2)`, `(x3, y3)`.
- Center: The center point of the tag in the image: `(x, y)`.
- Pose: The 3D position and orientation of the tag relative to the camera.

> [!Note]
> For more information, including the paper, technical reference, visit the
>  [AprilTag official page](https://april.eecs.umich.edu/software/apriltag.html)



## Usage

### RealSense Version
The RealSense version uses parameters defined in `src/utils.py` and accepts additional arguments:
- `camera_offset` and `R_bc`: These represent the coordinate transformation from the desired base frame to the camera frame for detection. They are hardcoded in the code and can be modified directly at [utils.py](https://github.com/RHplusLab/RHp_apriltag_ros2/blob/733f90c5955b7042bafa38364e85f1ebb754c1ac/src/rhp_apriltag_ros2/utils.py#L130)
- `surface_offset`: This allows fine-tuning of the robot base in the x and y directions after transformation, specified as an argument during launch.
  
To enhance detection accuracy, a calibration step has been implemented by intersecting ground truth (g.t) coordinates with detected values. This process refines the transformation and is implemented in the code as shown below [see line 167 in utils.py](https://github.com/RHplusLab/RHp_apriltag_ros2/blob/733f90c5955b7042bafa38364e85f1ebb754c1ac/src/rhp_apriltag_ros2/utils.py#L167):


To help visualize the relationship between these parameters and the system, the following diagram illustrates the coordinate transformation process:

<div align="center">
  <img src="https://github.com/user-attachments/assets/bc97050e-2eb4-4521-b908-b738a90f80dd" width="50%">
</div>

Launch example:

```
ros2 launch rhp_apriltag_ros2 apriltag_realsense.launch.py  surface_offset:=(5.0,6.0)
```

### Webcam Version
For webcam-based detection, use the following launch command:
```
ros2 launch rhp_apriltag_ros2 apriltag_webcam.launch.py
```


### Published Topics
The system publishes the following ROS 2 topics. For detailed message definitions, refer to the [RHp_apriltag_msgs repository](https://github.com/RHplusLab/RHp_apriltag_msgs).

- `/apriltag_detections` (type: `AprilTagDetectionArray`)
  - Contains an array of detected AprilTags with their IDs, corners, and poses.
- `/apriltag_image` (type: `sensor_msgs/Image`)
  - Publishes the image with AprilTag detections overlaid.

### Additional Resources
For a deeper understanding of the AprilTag detection concepts and related code, check out the [summary_theory_with_code.pdf](https://github.com/RHplusLab/rhp_apriltag_ros2/blob/main/docs/summary_theory_with_code.pdf) in the repository.
