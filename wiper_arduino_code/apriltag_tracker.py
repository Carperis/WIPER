import numpy as np
import cv2
import pyrealsense2 as rs
import threading
import time
import math
from pupil_apriltags import Detector
from shared_state import state as shared_state, state_lock as shared_state_lock


class AprilTagTracker(threading.Thread):
    def __init__(self, tag_size=0.041, tag_robot=0, tag_ref=1, state_ref=None):
        super().__init__()
        self.daemon = True
        self.tag_size = tag_size
        self.tag_robot = tag_robot
        self.tag_ref = tag_ref
        self.state = shared_state
        self.running = True

        # Setup camera
        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            self.pipeline.start(config)
            print("[INFO] AprilTag tracker started.")
            self.camera_available = True
        except RuntimeError:
            print("[WARN] No camera connected. Tracker disabled.")
            self.camera_available = False


        # Align to color stream
        self.align = rs.align(rs.stream.color)

        # Setup AprilTag detector
        self.detector = Detector(
            families='tagStandard41h12',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # Camera intrinsics will be set later
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((4, 1))

    def stop(self):
        self.running = False
        self.pipeline.stop()

    def run(self):
        while self.running:
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            if not color_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            if self.camera_matrix is None:
                intr = color_frame.profile.as_video_stream_profile().intrinsics
                self.camera_matrix = np.array([[intr.fx, 0, intr.ppx],
                                               [0, intr.fy, intr.ppy],
                                               [0, 0, 1]])

            detections = self.detector.detect(
                gray_image,
                estimate_tag_pose=True,
                camera_params=self._get_cam_params(),
                tag_size=self.tag_size
            )

            tag_dict = {d.tag_id: d for d in detections}
            if self.tag_robot not in tag_dict or self.tag_ref not in tag_dict:
                continue  # Both tags required

            pose_robot = tag_dict[self.tag_robot]
            pose_ref = tag_dict[self.tag_ref]

            rvec_robot = pose_robot.pose_R
            tvec_robot = pose_robot.pose_t.reshape(3, 1)

            rvec_ref = pose_ref.pose_R
            tvec_ref = pose_ref.pose_t.reshape(3, 1)

            x, y, theta = self._compute_relative_pose(rvec_robot, tvec_robot, rvec_ref, tvec_ref)

            now = time.time()
            with shared_state_lock:
                if not self.state.get('cam_pose_initialized', False):
                    self.state['cam_pose_initialized'] = True
                    self.state['last_cam_x'] = x
                    self.state['last_cam_y'] = y
                    self.state['last_cam_theta'] = theta
                    self.state['cam_x_acc'] = 0.0
                    self.state['cam_y_acc'] = 0.0
                    self.state['cam_theta_acc'] = 0.0
                else:
                    dx = x - self.state['last_cam_x']
                    dy = y - self.state['last_cam_y']
                    dtheta = (theta - self.state['last_cam_theta'] + 540) % 360 - 180  # shortest angle

                    # Accumulate
                    self.state['cam_x_acc'] += dx
                    self.state['cam_y_acc'] += dy

                    # Update last pose
                    self.state['last_cam_x'] = x
                    self.state['last_cam_y'] = y

                self.state['cam_x'] = self.state['cam_x_acc']
                self.state['cam_y'] = self.state['cam_y_acc']
                self.state['cam_theta'] = theta
                self.state['cam_timestamp'] = now

            #print(f"[APRILTAG] x: {x:.3f}, y: {y:.3f}, θ: {theta:.2f}, ts: {now}")

    def _get_cam_params(self):
        fx, fy = self.camera_matrix[0, 0], self.camera_matrix[1, 1]
        cx, cy = self.camera_matrix[0, 2], self.camera_matrix[1, 2]
        return (fx, fy, cx, cy)

    def _compute_relative_pose(self, rvec_robot, tvec_robot, rvec_ref, tvec_ref):
        """Convert robot tag pose into reference tag's frame."""
        R_ref_inv = rvec_ref.T
        t_ref_inv = -R_ref_inv @ tvec_ref

        t_robot_world = R_ref_inv @ tvec_robot + t_ref_inv
        R_robot_world = R_ref_inv @ rvec_robot

        x = t_robot_world[0, 0]
        y = -t_robot_world[1, 0]
        yaw = math.atan2(R_robot_world[1, 0], R_robot_world[0, 0])
        theta = np.degrees(yaw) % 360

        return x, y, theta
