#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import can
import struct
import pyrealsense2 as rs
import numpy as np
from std_msgs.msg import Float32
from ultralytics import YOLO
import cv2
import os
import time
from datetime import datetime
from enum import Enum

class RobotState(Enum):
    NORMAL_DRIVING = "normal"
    AVOIDING_PILLAR = "avoiding_pillar"
    BACKOFF_MANEUVER = "backoff"
    WALL_FOLLOWING = "wall_following"
    TURNED = "turned"
    STOPPED = "stopped"

class MotorServoController(Node):
    # Constants
    FRONT_AVOID_MIN = 0.30
    FRONT_AVOID_MAX = 0.40
    MAX_SPEED = 2000
    MIN_SERVO = 25
    MAX_SERVO = 155
    YOLO_CONFIDENCE = 0.5
    WATCHDOG_TIMEOUT = 1.0
    FPS_LOG_INTERVAL = 100

    def __init__(self):
        super().__init__('motor_servo_controller')

        # ---------- CAN bus ----------
        try:
            self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=250000)
        except Exception:
            self.get_logger().warning('⚠️ CAN bus not available - commands will be no-ops')
            self.bus = None

        # ---------- Speed / steering ----------
        self.normal_speed = 750
        self.obstacle_speed = 750
        self.turn_speed = 750
        self.desired_speed = self.normal_speed
        self.servo_angle = 90

        # ---------- Wall following ----------
        self.target_distance = 0.65  # right wall target (m)
        self.Kp = 70

        # ---------- Front obstacle ----------
        self.front_threshold = 01.0    # general "too close" threshold (m)
        self.front_max_range = 1.5    # cap far depth for median (m)

        # ---------- ROI & camera ----------
        self.roi_height_px = 10
        self.roi_width_px = 90
        self.front_roi_height_px = 20

        # ---------- Three-point turn state (disabled below) ----------
        self.performing_three_point_turn = False
        self.three_point_turn_stage = 0
        self.three_point_turn_start_time = None

        # ---------- Backoff maneuver (now 2 stages: pre-turn left, then reverse) ----------
        self.performing_backoff = False
        self.backoff_stage = 0            # 0: idle, 1: pre-turn, 2: reverse
        self.backoff_start_time = None

        # Backoff timings
        self.preturn_duration = 1.3      # seconds
        self.preturn_servo = 145         # deg (right) - turn away from right wall
        self.preturn_speed = 450         # forward at low speed

        # Stage 2: reverse settings
        self.backoff_duration = 2.4      # seconds
        self.backoff_servo = 60          # deg while reversing (left) - continue away from right wall
        self.backoff_speed = -1100       # reverse speed

        # Stage 3: center settings  
        self.center_duration = 0.5        # time to center steering
        self.center_servo = 90           # straight

        # ---------- Pillar avoidance tracking ----------
        self.current_avoiding_pillar = None

        self.pillar_avoidance_frames = 0
        self.max_frames_without_pillar = 15
        self._last_valid_pillars = []  # Store pillars with zone info

        # ---------- RealSense ----------
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(config)
        profile = self.pipeline.get_active_profile()
        self.depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)

        # ---------- YOLO ----------
        # allow overriding model path via ROS parameter for easier tuning
        self.declare_parameter('yolo_model_path', '/home/aupp/ros2_ws/src/motor_servo_controller/motor_servo_controller/best.pt')
        model_path = self.get_parameter('yolo_model_path').value
        if not os.path.exists(model_path):
            self.get_logger().warning(f"YOLO model not found at {model_path} — attempting default path")
            model_path = '/home/aupp/ros2_ws/src/motor_servo_controller/motor_servo_controller/best.pt'
        self.yolo_model = YOLO(model_path)
        self.get_logger().info(f"YOLO class names: {self.yolo_model.names}")

        # ---------- Timers / subscriptions ----------
        self.timer = self.create_timer(0.1, self.send_command)
        self.subscription = self.create_subscription(Float32, '/bno055/yaw_deg', self.yaw_callback, 10)

        # ---------- Lap tracking ----------
        self.passed_90 = False
        self.passed_180 = False
        self.passed_270 = False
        self.rotation_count = 0
        self.robot_stopped = False

        # ---------- First-lap helpers ----------
        self.first90_active = False
        self.first90_done = False
        self.slowdown_done = False
        self.base_speed = self.desired_speed

        # ---------- Safety & Monitoring ----------
        self.current_state = RobotState.NORMAL_DRIVING
        self.last_command_time = time.time()
        self.frame_count = 0
        self.detection_count = 0
        self.last_fps_time = time.time()

        # ---------- Corner Turn State ----------
        self.corner_turn_start_time = None
        self.traveled_in_new_section = 0.0
        self.wiggle_start_time = None
        self.target_speed_for_ramp = 0.9
        self.ramp_tau = 0.7
        self.ramp_start_time = None
        self.ramp_start_speed = None
        
        # Corner turn tuning parameters
        self.corner_probe_min_speed = 300  # minimum probe speed 
        self.corner_probe_max_speed = 450  # maximum probe speed
        self.corner_wiggle_amp_deg = 8     # wiggle amplitude in degrees
        self.corner_wiggle_hz = 1.5        # wiggle frequency
        self.corner_min_travel_dist = 0.30 # minimum travel before transition
        self.corner_front_clear_threshold = 0.8  # front distance to detect corner
        self.corner_sharp_turn_threshold = 35    # steering angle to trigger corner detection

        # ---------- Timers / subscriptions ----------
        # Only create each timer/subscription ONCE
        self.watchdog_timer = self.create_timer(0.5, self.watchdog_check)

        self.get_logger().info("✅ Motor & Servo controller with color avoidance + enhanced safety features.")

    def watchdog_check(self):
        """Emergency stop if main loop stops responding"""
        if time.time() - self.last_command_time > self.WATCHDOG_TIMEOUT:
            self.get_logger().error("🚨 Watchdog timeout - stopping robot!")
            self.current_state = RobotState.STOPPED
            self.stop()

    # ---------- Corner Turn Helper Functions ----------
    def analyze_corner_rois(self, depth_image):
        """Analyze ROIs for corner turn logic (848x480 assumed)"""
        try:
            height, width = depth_image.shape
            
            # ROIs based on your pseudocode
            # F = med(depth[160:220,320:520])
            F_roi = depth_image[160:220, 320:520]
            F_valid = F_roi[F_roi > 0]
            F = np.median(F_valid) * self.depth_scale if F_valid.size > 0 else None
            
            # S1 = med(depth[170:250,560:600]) - inner-edge sentinels
            S1_roi = depth_image[170:250, 560:600]
            S1_valid = S1_roi[S1_roi > 0]
            S1 = np.median(S1_valid) * self.depth_scale if S1_valid.size > 0 else None
            
            # S2 = med(depth[170:250,600:640])
            S2_roi = depth_image[170:250, 600:640]
            S2_valid = S2_roi[S2_roi > 0]
            S2 = np.median(S2_valid) * self.depth_scale if S2_valid.size > 0 else None
            
            return F, S1, S2
        except Exception as e:
            self.get_logger().error(f"Error in analyze_corner_rois: {e}")
            return None, None, None

    def raycast_diagonal_visibility(self, depth_image, heading_deg=40, dmax=1.2):
        """Raycast to check diagonal visibility into corner"""
        try:
            height, width = depth_image.shape
            center_x, center_y = width // 2, height // 2
            
            heading_rad = np.deg2rad(heading_deg)
            max_steps = int(dmax / self.depth_scale / 0.01)  # step size ~1cm
            
            for step in range(1, max_steps):
                x = int(center_x + step * np.cos(heading_rad) * 10)
                y = int(center_y + step * np.sin(heading_rad) * 10)
                
                if 0 <= x < width and 0 <= y < height:
                    depth_val = depth_image[y, x] * self.depth_scale
                    if depth_val > 0 and depth_val < step * 0.01:
                        return step * 0.01  # obstacle detected
                else:
                    break
            
            return dmax  # no obstacle found within range
        except Exception:
            return 0.0

    def calculate_coverage_ratio(self, depth_image):
        """Calculate valid depth ratio in right half of image"""
        try:
            height, width = depth_image.shape
            right_half = depth_image[:, 320:640]
            total_pixels = right_half.size
            valid_pixels = np.count_nonzero(right_half)
            return valid_pixels / total_pixels if total_pixels > 0 else 0.0
        except Exception:
            return 0.0

    def calculate_wiggle_steering(self, amp_deg=None, hz=None):
        """Generate sinusoidal wiggle for steering"""
        if amp_deg is None:
            amp_deg = self.corner_wiggle_amp_deg
        if hz is None:
            hz = self.corner_wiggle_hz
            
        if self.wiggle_start_time is None:
            self.wiggle_start_time = time.time()
        
        t = time.time() - self.wiggle_start_time
        wiggle = np.deg2rad(amp_deg) * np.sin(2 * np.pi * hz * t)
        return wiggle

    def pd_side_control(self, right_dist):
        """PD control for side wall following"""
        if right_dist is None:
            return 0.0
        
        error = self.target_distance - right_dist
        offset_rad = np.deg2rad(self.Kp * error)
        return offset_rad

    def ramp_speed(self, target_speed, tau=0.7):
        """Exponential ramp to target speed"""
        current_time = time.time()
        
        if self.ramp_start_time is None:
            self.ramp_start_time = current_time
            self.ramp_start_speed = self.desired_speed
        
        elapsed = current_time - self.ramp_start_time
        alpha = 1 - np.exp(-elapsed / tau)
        ramped_speed = self.ramp_start_speed + alpha * (target_speed - self.ramp_start_speed)
        return ramped_speed

    def clamp_speed(self, speed, min_val, max_val):
        """Clamp speed to range"""
        return max(min_val, min(max_val, speed))

    def reset_corner_turn_state(self):
        """Reset all corner turn related state variables"""
        self.corner_turn_start_time = None
        self.traveled_in_new_section = 0.0
        self.wiggle_start_time = None
        self.ramp_start_time = None
        self.ramp_start_speed = None

    def handle_corner_turn_logic(self, depth_image, right_dist):
        """Implement corner turn logic from pseudocode"""
        try:
            F, S1, S2 = self.analyze_corner_rois(depth_image)
            visible_diag = self.raycast_diagonal_visibility(depth_image, heading_deg=40, dmax=1.2)
            coverage = self.calculate_coverage_ratio(depth_image)
            
            if self.current_state == RobotState.TURNED:
                # Initialize corner turn timing if needed
                if self.corner_turn_start_time is None:
                    self.corner_turn_start_time = time.time()
                    self.traveled_in_new_section = 0.0
                    self.wiggle_start_time = None
                
                # Estimate distance traveled (rough approximation)
                dt = 0.1  # control loop period
                current_speed_ms = abs(self.desired_speed) / 1000.0  # rough conversion
                self.traveled_in_new_section += current_speed_ms * dt
                
                # Probe speed cap
                probe_speed = self.clamp_speed(self.normal_speed, self.corner_probe_min_speed, self.corner_probe_max_speed)
                
                # PD control + wiggle
                pd_offset = self.pd_side_control(right_dist)
                wiggle_offset = self.calculate_wiggle_steering()
                total_steering_offset = pd_offset + wiggle_offset
                
                # Convert to servo angle
                self.servo_angle = 90 + np.rad2deg(total_steering_offset)
                self.servo_angle = max(self.MIN_SERVO, min(self.MAX_SERVO, self.servo_angle))
                
                # Calculate required distance for safety
                v_ms = probe_speed / 1000.0
                d_req = v_ms * 0.15 + (v_ms * v_ms) / (2 * 1.8) + 0.33
                
                # Check clearance conditions
                min_sentinel = min(S1, S2) if S1 is not None and S2 is not None else 0.0
                clear_ok = (visible_diag >= d_req and 
                           coverage >= 0.45 and 
                           min_sentinel >= 0.8)
                
                if clear_ok and self.traveled_in_new_section >= self.corner_min_travel_dist:
                    # Transition back to wall following
                    self.desired_speed = self.ramp_speed(900, tau=0.7)  # ramp to 0.9 scaled
                    self.current_state = RobotState.WALL_FOLLOWING
                    self.reset_corner_turn_state()
                    self.get_logger().info(f"🔄 Corner clear! Transitioning to WALL_FOLLOWING. traveled={self.traveled_in_new_section:.2f}m")
                else:
                    self.desired_speed = probe_speed
                    self.get_logger().debug(f"🌀 TURNED: F={F:.2f}, S1={S1:.2f}, S2={S2:.2f}, vis_diag={visible_diag:.2f}, cov={coverage:.2f}, traveled={self.traveled_in_new_section:.2f}")
                
                # Fallback safety check
                if F is not None and F < 0.55 and (min_sentinel < 0.8 or coverage < 0.4):
                    self.desired_speed = 0
                    self.servo_angle = 90
                    self.get_logger().warning("🚨 Corner turn safety stop - front blocked!")
                
                return True  # Handled by corner turn logic
                
        except Exception as e:
            self.get_logger().error(f"Error in corner turn logic: {e}")
            
        return False  # Not handled by corner turn logic

    def send_command(self):
        """Main control loop with safety checks"""
        # Update watchdog
        self.last_command_time = time.time()
        self.frame_count += 1

        # Log FPS periodically
        if self.frame_count % self.FPS_LOG_INTERVAL == 0:
            now = time.time()
            fps = self.FPS_LOG_INTERVAL / (now - self.last_fps_time)
            self.get_logger().info(f"📊 FPS: {fps:.1f}, Detections: {self.detection_count}")
            self.last_fps_time = now

        # Check if robot should be stopped
        if self.robot_stopped or self.current_state == RobotState.STOPPED:
            return

        # Run timed backoff (two-stage) if active
        if self.performing_backoff:
            self.handle_backoff_maneuver()
            return

        # ----- Get frames -----
        try:
            frames = self.pipeline.wait_for_frames()
            frames = self.align.process(frames)
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            if not depth_frame or not color_frame:
                return
        except Exception as e:
            self.get_logger().error(f"Error getting frames: {e}")
            return

        # Compute ROIs
        left_dist, front_dist, right_dist, color_image, _ = self.process_frames(depth_frame, color_frame)

        # Get depth image for corner turn analysis
        depth_image = np.asanyarray(depth_frame.get_data())

        # ----- Corner Turn Logic -----
        if self.current_state == RobotState.TURNED:
            if self.handle_corner_turn_logic(depth_image, right_dist):
                self._send_can(self.desired_speed, self.servo_angle)
                return

        # Detect pillar color and get valid pillars with zone information
        detected_color = self.detect_pillar_color(color_image)

        # Get all valid pillars from the last detection
        valid_pillars = getattr(self, '_last_valid_pillars', [])

        # ----- Trigger the two-stage front-avoid when NOT seeing a color pillar -----
        if detected_color is None and front_dist is not None and self.FRONT_AVOID_MIN <= front_dist <= self.FRONT_AVOID_MAX:
            self.start_backoff_maneuver()
            return

        # ----- Zone-based pillar avoidance -----
        if detected_color and valid_pillars:
            self.current_state = RobotState.AVOIDING_PILLAR
            servo_angle, action = self.get_pillar_action(valid_pillars)
            if servo_angle is not None:
                self.servo_angle = servo_angle
                self.desired_speed = self.obstacle_speed
        elif detected_color == "red":
            # Fallback for red pillar without zone info
            self.servo_angle = 130   # steer right
            self.desired_speed = self.obstacle_speed
            self.get_logger().info("🟥 Red pillar detected → Avoiding right (servo=130°) + SLOW DOWN.")
        elif detected_color == "green":
            # Fallback for green pillar without zone info
            self.servo_angle = 55  # steer left
            self.desired_speed = self.obstacle_speed
            self.get_logger().info(f"🟩 Green pillar detected → Avoiding left (servo={self.servo_angle}°) + SLOW DOWN.")
        elif detected_color is None and left_dist is not None and left_dist < 0.5:  # only when no pillar detected
            self.servo_angle = 105  # moderate right turn to avoid left wall
            self.desired_speed = self.turn_speed
            self.get_logger().info(f"⚠️ Left wall too close ({left_dist:.2f}m) → Turning right (servo=105°)")
        elif right_dist is not None:
            self.current_state = RobotState.WALL_FOLLOWING
            
            # Check for corner turn condition - when making a sharp turn
            if abs(self.servo_angle - 90) > self.corner_sharp_turn_threshold:
                # Additional checks for corner detection
                F, S1, S2 = self.analyze_corner_rois(depth_image)
                if F is not None and F > self.corner_front_clear_threshold:  # front is clear, might be entering corner
                    self.current_state = RobotState.TURNED
                    self.reset_corner_turn_state()  # Reset state for new corner
                    self.get_logger().info(f"🔄 Sharp turn detected (servo={self.servo_angle:.1f}°) with clear front → Entering TURNED state")
                    if self.handle_corner_turn_logic(depth_image, right_dist):
                        self._send_can(self.desired_speed, self.servo_angle)
                        return
            
            if 0.3 <= right_dist <= 0.8:
                error = self.target_distance - right_dist
                offset = -self.Kp * error  # Negative for right wall (opposite direction)
                self.servo_angle = 90 + offset
                self.servo_angle = max(self.MIN_SERVO, min(self.MAX_SERVO, self.servo_angle))
                if abs(self.servo_angle - 90) > 20:
                    self.desired_speed = self.turn_speed
                    self.get_logger().debug(f"📐 RightDist={right_dist:.2f}m | Servo={self.servo_angle:.1f} + TURN SLOW")
                else:
                    self.desired_speed = self.normal_speed
                    self.get_logger().debug(f"📐 RightDist={right_dist:.2f}m | Servo={self.servo_angle:.1f}")
            else:
                self.servo_angle = 90
                self.desired_speed = self.normal_speed
                self.get_logger().debug(f"↔️ RightDist={right_dist:.2f}m out of [0.3, 0.8] → Going straight.")
        else:
            self.current_state = RobotState.NORMAL_DRIVING
            self.servo_angle = 90
            self.desired_speed = self.normal_speed
            self.get_logger().debug("➡️ Right wall not detected → Going straight.")

        # ----- Send CAN -----
        self._send_can(self.desired_speed, self.servo_angle)

    # ---------- NEW: Backoff helpers (two-stage) ----------
    def start_backoff_maneuver(self):
        """Begin two-stage avoid: Stage1 pre-turn-right, Stage2 reverse."""
        self.performing_backoff = True
        self.backoff_stage = 1
        self.backoff_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(f"↩️ Front wall in window → Stage1: pre-turn RIGHT {self.preturn_servo}° for {self.preturn_duration:.1f}s, then Stage2: reverse {self.backoff_duration:.1f}s (servo={self.backoff_servo}°).")

    def handle_backoff_maneuver(self):
        """Run Stage1 (right pre-turn) then Stage2 (reverse with steering)."""
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - (self.backoff_start_time or now)

        if self.backoff_stage == 1:
            # Stage 1: pre-turn RIGHT
            self.servo_angle = self.preturn_servo
            self.desired_speed = self.preturn_speed  # gentle forward
            self._send_can(self.desired_speed, self.servo_angle)
            self.get_logger().info(f"↩️ Backoff Stage1 (pre-turn): Servo={self.servo_angle}°, Speed={self.desired_speed}, t={elapsed:.2f}s")
            if elapsed >= self.preturn_duration:
                # move to Stage 2
                self.backoff_stage = 2
                self.backoff_start_time = now  # reset timer for Stage 2
        elif self.backoff_stage == 2:
            # Stage 2: reverse with steering
            self.servo_angle = self.backoff_servo
            self.desired_speed = self.backoff_speed   # negative = reverse
            self._send_can(self.desired_speed, self.servo_angle)
            self.get_logger().info(f"↩️ Backoff Stage2 (reverse): Servo={self.servo_angle}°, Speed={self.desired_speed}, t={elapsed:.2f}s")
            if elapsed >= self.backoff_duration:
                # move to Stage 3
                self.backoff_stage = 3
                self.backoff_start_time = now  # reset timer for Stage 3
        elif self.backoff_stage == 3:
            # Stage 3: center steering before resuming
            self.servo_angle = self.center_servo
            self.desired_speed = self.normal_speed
            self._send_can(self.desired_speed, self.servo_angle)
            self.get_logger().info(f"↩️ Backoff Stage3 (center): Servo={self.servo_angle}°, Speed={self.desired_speed}, t={elapsed:.2f}s")
            if elapsed >= self.center_duration:
                # Done
                self.performing_backoff = False
                self.backoff_stage = 0
                self.servo_angle = 90
                self.desired_speed = self.normal_speed
                self.get_logger().info("✅ Backoff complete — resuming normal operation.")

    # ---------- Depth ROIs ----------
    def process_frames(self, depth_frame, color_frame):
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        height, width = depth_image.shape

        # Left ROI
        target_row = int(height * 0.55)
        roi_x = 0
        roi_y = max(0, target_row - self.roi_height_px // 2)
        left_roi = depth_image[roi_y:roi_y + self.roi_height_px, roi_x:roi_x + self.roi_width_px]
        left_valid = left_roi[left_roi > 0]
        left_dist = np.median(left_valid) * self.depth_scale if left_valid.size > 0 else None

        # Front ROI
        front_x = (width - self.roi_width_px) // 2
        front_y = int(height * 0.46)
        front_roi = depth_image[front_y:front_y + self.front_roi_height_px,
                                front_x:front_x + self.roi_width_px]
        front_valid = front_roi[(front_roi > 0) &
                                (front_roi < (self.front_max_range / self.depth_scale))]
        front_dist = np.median(front_valid) * self.depth_scale if front_valid.size > 0 else None

        # Right ROI (fixed right_dist bug)
        right_x = width - self.roi_width_px
        right_y = max(0, target_row - self.roi_height_px // 2)
        right_roi = depth_image[right_y:right_y + self.roi_height_px, right_x:width]
        right_valid = right_roi[right_roi > 0]
        right_dist = np.median(right_valid) * self.depth_scale if right_valid.size > 0 else None

        return left_dist, front_dist, right_dist, color_image, None

    # ---------- YOLO color detection with zones ----------
    def detect_pillar_color(self, color_image):
        """Detect pillar color using YOLO with improved error handling"""
        try:
            results = self.yolo_model(color_image, conf=self.YOLO_CONFIDENCE)
            boxes = results[0].boxes
            class_names = results[0].names
            detected_color = None

            # For optional saving/annotation
            annotated_image = color_image.copy()
            detection_found = False
            valid_detection_found = False

            # Get a depth frame (note: not strictly synchronized with 'color_image')
            frames = self.pipeline.wait_for_frames()
            frames = self.align.process(frames)
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                self.get_logger().warning("⚠️ No depth frame available for pillar detection")
                return None
            depth_image = np.asanyarray(depth_frame.get_data())

            # Zone definitions - divide image into 3 zones
            image_width = color_image.shape[1]
            # Make the middle zone smaller (centered). Adjust this fraction to change middle width.
            middle_width_frac = 0.40  # 30% middle zone (was 50%)
            left_zone_end = int(image_width * (0.5 - middle_width_frac / 2))
            right_zone_start = int(image_width * (0.5 + middle_width_frac / 2))
            # Middle zone is between left_zone_end and right_zone_start (now smaller)

            valid_pillars = []

            if boxes.cls is not None and len(boxes.cls) > 0:
                self.get_logger().info(f"🔍 YOLO found {len(boxes.cls)} detections")
                for i, (cls_idx, conf) in enumerate(zip(boxes.cls.cpu().numpy(), boxes.conf.cpu().numpy())):
                    class_name = class_names[int(cls_idx)]
                    self.get_logger().info(f"  Detection {i+1}: {class_name} with {conf:.2f} confidence")

                    if conf < 0.5:
                        self.get_logger().info(f"    ❌ Skipped: confidence {conf:.2f} < 0.5")
                        continue

                    detection_found = True

                    if boxes.xyxy is not None and i < len(boxes.xyxy):
                        x1, y1, x2, y2 = boxes.xyxy[i].cpu().numpy().astype(int)
                        box_width = x2 - x1
                        box_height = y2 - y1
                        box_area = box_width * box_height
                        aspect_ratio = box_height / box_width if box_width > 0 else 0

                        center_x = int((x1 + x2) / 2)
                        center_y = int((y1 + y2) / 2)

                        if (0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]):
                            sample_size = min(10, box_width // 4, box_height // 4)
                            x_start = max(0, center_x - sample_size)
                            x_end = min(depth_image.shape[1], center_x + sample_size)
                            y_start = max(0, center_y - sample_size)
                            y_end = min(depth_image.shape[0], center_y + sample_size)

                            depth_sample = depth_image[y_start:y_end, x_start:x_end]
                            valid_depths = depth_sample[depth_sample > 0]

                            if valid_depths.size > 5:
                                object_distance = np.percentile(valid_depths, 50) * self.depth_scale
                                depth_std = np.std(valid_depths) * self.depth_scale
                                self.get_logger().info(f"    📏 Depth: {object_distance:.2f}m (std: {depth_std:.3f}m, {valid_depths.size} points)")
                            else:
                                self.get_logger().info(f"    ❌ Insufficient depth data: {valid_depths.size} points")
                                continue
                        else:
                            continue

                        max_detection_distance = 0.70
                        min_area = 300
                        max_area = 50000
                        min_aspect_ratio = 0.3
                        max_aspect_ratio = 5.0

                        is_valid_pillar = (
                            object_distance <= max_detection_distance and
                            min_area <= box_area <= max_area and
                            min_aspect_ratio <= aspect_ratio <= max_aspect_ratio and
                            box_width >= 15 and box_height >= 20
                        )

                        pillar_color = None
                        if 'red' in class_name.lower():
                            pillar_color = "red"
                        elif 'green' in class_name.lower():
                            pillar_color = "green"

                        if is_valid_pillar and pillar_color:
                            # Determine which zone the pillar is in
                            if center_x <= left_zone_end:
                                zone = "left"
                            elif center_x >= right_zone_start:
                                zone = "right"
                            else:
                                zone = "middle"

                            valid_pillars.append({
                                'color': pillar_color,
                                'distance': object_distance,
                                'area': box_area,
                                'coords': (x1, y1, x2, y2),
                                'center_x': center_x,
                                'center_y': center_y,
                                'conf': conf,
                                'class_name': class_name,
                                'zone': zone
                            })

                        # (Optional) draw annotations/saving with zone info
                        color = (0, 0, 255) if pillar_color == "red" else ((0, 255, 0) if pillar_color == "green" else (255, 255, 0))
                        if not is_valid_pillar:
                            color = (128, 128, 128)
                        cv2.rectangle(annotated_image, (x1, y1), (x2, y2), color, 2)

                        if object_distance > max_detection_distance:
                            status = "TOO FAR"
                        elif is_valid_pillar:
                            # Determine zone for status
                            if center_x <= left_zone_end:
                                zone_name = "LEFT"
                            elif center_x >= right_zone_start:
                                zone_name = "RIGHT"
                            else:
                                zone_name = "MIDDLE"
                            status = f"VALID-{zone_name}"
                        else:
                            status = "INVALID SIZE"

                        label = f"{class_name}: {conf:.2f} [{status}]"
                        info_text = f"Dist:{object_distance:.2f}m Area:{box_area:.0f}"
                        label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)[0]
                        cv2.rectangle(annotated_image, (x1, y1 - label_size[1] - 25),
                                      (x1 + max(label_size[0], 200), y1), color, -1)
                        cv2.putText(annotated_image, label, (x1, y1 - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                        cv2.putText(annotated_image, info_text, (x1, y1 - 5),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

            # Draw zone lines for visualization
            cv2.line(annotated_image, (left_zone_end, 0), (left_zone_end, annotated_image.shape[0]), (255, 255, 255), 1)
            cv2.line(annotated_image, (right_zone_start, 0), (right_zone_start, annotated_image.shape[0]), (255, 255, 255), 1)
            cv2.putText(annotated_image, "LEFT", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(annotated_image, "MIDDLE", (int(image_width*0.45), 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(annotated_image, "RIGHT", (right_zone_start + 10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Choose closest valid pillar and apply sticky logic
            if valid_pillars:
                # Store valid pillars for zone-based logic
                self._last_valid_pillars = valid_pillars

                valid_pillars.sort(key=lambda p: p['distance'])
                if self.current_avoiding_pillar is not None:
                    still = any(p['color'] == self.current_avoiding_pillar for p in valid_pillars)
                    if still:
                        detected_color = self.current_avoiding_pillar
                        valid_detection_found = True
                        self.pillar_avoidance_frames = 0
                        closest_pillar = next(p for p in valid_pillars if p['color'] == self.current_avoiding_pillar)
                        closest_distance = closest_pillar['distance']
                        closest_zone = closest_pillar['zone']
                        self.get_logger().info(f"🔄 Continuing to avoid {self.current_avoiding_pillar} pillar in {closest_zone} zone at {closest_distance:.2f}m")
                    else:
                        self.pillar_avoidance_frames += 1
                        if self.pillar_avoidance_frames >= self.max_frames_without_pillar:
                            self.current_avoiding_pillar = None
                            self.pillar_avoidance_frames = 0
                            closest_pillar = valid_pillars[0]
                            detected_color = closest_pillar['color']
                            self.current_avoiding_pillar = detected_color
                            valid_detection_found = True
                            self.get_logger().info(f"✅ Finished avoiding previous pillar, now avoiding closest {detected_color} pillar in {closest_pillar['zone']} zone at {closest_pillar['distance']:.2f}m")
                        else:
                            self.get_logger().info(f"⏳ Cooldown: {self.pillar_avoidance_frames}/{self.max_frames_without_pillar} frames since last pillar")
                else:
                    closest_pillar = valid_pillars[0]
                    detected_color = closest_pillar['color']
                    self.current_avoiding_pillar = detected_color
                    valid_detection_found = True
                    self.get_logger().info(f"🎯 New pillar detected: {detected_color} in {closest_pillar['zone']} zone at {closest_pillar['distance']:.2f}m (closest of {len(valid_pillars)} pillars)")
            else:
                # Clear stored pillars when none detected
                self._last_valid_pillars = []

                if self.current_avoiding_pillar is not None:
                    self.pillar_avoidance_frames += 1
                    if self.pillar_avoidance_frames >= self.max_frames_without_pillar:
                        self.current_avoiding_pillar = None
                        self.pillar_avoidance_frames = 0
                        self.get_logger().info("🏁 Finished avoiding pillar - ready for new detections")

            if detection_found:
                self.detection_count += 1  # Update detection counter
                # Removed image saving

            return detected_color if valid_detection_found else None

        except Exception as e:
            self.get_logger().error(f"🚨 YOLO detection error: {e}")
            return None

    # ---------- Zone-based pillar avoidance ----------
    def get_pillar_action(self, valid_pillars):
        """
        Determine steering action based on pillar color and zone.
        Algorithm:
        - Green pillar in middle zone: turn moderately left
        - Green pillar in right zone: do not turn (too far right)
        - Red pillar in middle zone: turn moderately right  
        - Red pillar in left zone: do not turn (too far left)
        """
        if not valid_pillars:
            return None, None

        # Sort by distance to get closest pillar
        valid_pillars.sort(key=lambda p: p['distance'])
        closest_pillar = valid_pillars[0]

        color = closest_pillar['color']
        zone = closest_pillar['zone']
        distance = closest_pillar['distance']

        self.get_logger().info(f"🎯 Closest pillar: {color} in {zone} zone at {distance:.2f}m")

        # Green pillar logic
        if color == "green":
            if zone == "middle":
                # Green in middle: turn moderately left
                servo_angle = 55  # moderate left turn
                action = "turn_left_moderate"
                self.get_logger().info("🟩 Green pillar in MIDDLE zone → Turn moderately LEFT")
            elif zone == "right":
                # Green on right: do not turn (too far right)
                servo_angle = 90  # go straight
                action = "no_turn_right"
                self.get_logger().info("🟩 Green pillar in RIGHT zone → Do NOT turn (too far right)")
            else:  # left zone
                # Green on left: turn left more aggressively
                servo_angle = 45  # more aggressive left turn
                action = "turn_left_aggressive"
                self.get_logger().info("🟩 Green pillar in LEFT zone → Turn LEFT aggressively")

        # Red pillar logic  
        elif color == "red":
            if zone == "middle":
                # Red in middle: turn moderately right
                servo_angle = 120  # moderate right turn
                action = "turn_right_moderate"
                self.get_logger().info("🟥 Red pillar in MIDDLE zone → Turn moderately RIGHT")
            elif zone == "left":
                # Red on left: do not turn (too far left)
                servo_angle = 90  # go straight
                action = "no_turn_left"
                self.get_logger().info("🟥 Red pillar in LEFT zone → Do NOT turn (too far left)")
            else:  # right zone
                # Red on right: turn right more aggressively
                servo_angle = 135  # more aggressive right turn
                action = "turn_right_aggressive"
                self.get_logger().info("🟥 Red pillar in RIGHT zone → Turn RIGHT aggressively")

        else:
            return None, None

        return servo_angle, action

    # ---------- Helpers ----------
    def _send_can(self, speed, servo_deg):
        """Send CAN message with safety checks and error handling"""
        try:
            # If CAN bus is not available, skip sending but log once
            if getattr(self, 'bus', None) is None:
                self.get_logger().debug('CAN bus unavailable - skipping send')
                return
            # Clamp values to safe ranges using constants
            speed = max(-self.MAX_SPEED, min(self.MAX_SPEED, int(speed)))
            servo_deg = max(self.MIN_SERVO, min(self.MAX_SERVO, int(servo_deg)))

            # Log extreme values for debugging
            if abs(speed) > 1500 or servo_deg < 40 or servo_deg > 140:
                self.get_logger().warning(f"⚠️ Extreme values: speed={speed}, servo={servo_deg}")

            servo_raw = int(servo_deg * 100)
            data = struct.pack(">ii", speed, servo_raw)
            msg = can.Message(arbitration_id=0x101, data=data, is_extended_id=False)
            self.bus.send(msg)

        except can.CanError as e:
            self.get_logger().error(f"🚨 CAN send failed: {e}")
            self.current_state = RobotState.STOPPED
            self.stop()  # Emergency stop on CAN failure
        except Exception as e:
            self.get_logger().error(f"🚨 Unexpected error in _send_can: {e}")
            self.stop()

    # ---------- Yaw / laps ----------
    def yaw_callback(self, msg):
        yaw = msg.data
        self.get_logger().info(f"🧭 Yaw: {yaw:.2f}°")

        # --- First-lap 0°→90° boost to 1700 ---
        if self.rotation_count == 0 and not self.robot_stopped:
            # Enter boost window (only once on first lap)
            if (0.0 <= yaw <= 90.0) and not self.first90_done and not self.first90_active:
                sign = 1 if self.base_speed < 0 else 1
                self.desired_speed = sign * 1300
                self.first90_active = True
                self.get_logger().info("🚀 First lap 0–90° → speed set to 1700.")

        # Sector markers (your original thresholds)
        if 80 <= yaw <= 100:
            self.passed_90 = True
            # Leave boost window once we've passed 90°, restore baseline
            if self.first90_active and not self.first90_done:
                self.desired_speed = self.base_speed
                self.first90_active = False
                self.first90_done = True
                self.get_logger().info("↩️ Passed 90° on first lap → restoring baseline speed.")
        elif 170 <= yaw <= 190:
            self.passed_180 = True
        elif 220 <= yaw <= 300:
            self.passed_270 = True

            # Third-lap slowdown at 270° (keeps your existing logic)
            if self.rotation_count == 2 and not self.slowdown_done and not self.robot_stopped:
                sign = 1 if self.desired_speed < 0 else 1
                self.desired_speed = sign * 300  # use 1000 if you prefer: sign * 1000
                self.slowdown_done = True
                self.get_logger().info("🕘 Third lap @270° → slowing to 700.")

        # Full-rotation check (unchanged)
        if self.passed_90 and self.passed_180 and self.passed_270:
            if yaw <= 10 or yaw >= 350:
                self.rotation_count += 1
                self.get_logger().info(f"🔁 Completed {self.rotation_count} full rotation(s).")
                self.passed_90 = self.passed_180 = self.passed_270 = False

                if self.rotation_count >= 3 and not self.robot_stopped:
                    self.get_logger().info("✅ Completed 3 full rotations — move forward before stopping.")
                    self.move_forward_for_duration()
                    self.robot_stopped = True

    # ---------- Stop helpers ----------
    def move_forward_for_duration(self, speed=500, duration_sec=0.5):
        self.get_logger().info(f"⏩ Moving forward for {duration_sec} sec before final stop...")
        self.desired_speed = speed
        self.servo_angle = 90
        self._send_can(self.desired_speed, self.servo_angle)
        self.sleep_for(duration_sec)
        self.stop()

    def sleep_for(self, seconds):
        start_time = self.get_clock().now().nanoseconds / 1e9
        while (self.get_clock().now().nanoseconds / 1e9) - start_time < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def stop(self):
        self.desired_speed = 0
        self.servo_angle = 90
        self._send_can(self.desired_speed, self.servo_angle)
        self.get_logger().info("🛑 Motors stopped, servo reset to 90°.")

    def destroy_node(self):
        # Ensure motors stopped before shutdown
        try:
            self.stop()
        except Exception:
            pass
        try:
            self.pipeline.stop()
        except Exception:
            pass
        try:
            if self.bus:
                self.bus.shutdown()
        except Exception:
            pass
        super().destroy_node()

    # Removed save_detection_image

def main(args=None):
    rclpy.init(args=args)
    node = MotorServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("🛑 Shutting down — sending stop command...")
        node.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()