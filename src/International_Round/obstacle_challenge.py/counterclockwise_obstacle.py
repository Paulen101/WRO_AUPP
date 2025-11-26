import os
import struct
import threading
import time
import numpy as np
import can
import cv2
import pyrealsense2 as rs
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from datetime import datetime
from queue import Queue
import math

# Suppress YOLO/Ultralytics verbose output
os.environ['YOLO_VERBOSE'] = 'False'
os.environ['ULTRALYTICS_VERBOSE'] = 'False'

from ultralytics import YOLO

class CombinedCornerReverseAndPillarAvoid(Node):

    # ---------------- Vision constants (subset of test_1.py) ----------------
    MODEL_PATH = "/home/aupp/ros2_ws/src/motor_servo_controller/motor_servo_controller/best.pt"
    CONF = 0.70
    IOU = 0.50

    # ROI and zone layout: 20% | 60% | 20%
    ROI_Y = (0.20, 0.85)  # Reduced from top: starts at 20% instead of 10%
    ZONE_LEFT_FRAC = 0.20
    ZONE_MIDDLE_FRAC = 0.80

    # Detection distance gating (meters)
    PILLAR_DETECT_DISTANCE = 2.0

    # --------------- LiDAR constants ---------------
    FRONT_DEG = (-15.0, +15.0)  # front sector for distance
    RANGE_MIN = 0.6
    RANGE_MAX = 4.0
    MIN_HITS = 10
    K_FRACT = 0.2

    # Obstacle detection (width-based)
    OBSTACLE_DETECT_DISTANCE = 0.8      # detect obstacles up to 0.8m away (increased from 0.6)
    WALL_WIDTH_THRESHOLD = 0.08         # width > threshold → wall (USER SET - DO NOT CHANGE!)
    
    # Corner detection thresholds (meters) - WIDENED for better detection
    FRONT_CORNER_WINDOW = (0.25, 0.40)  # front distance window to consider a corner (widened from 0.25-0.35)
    RIGHT_NEAR_THRESHOLD = 0.65         # right wall near enough means likely corner (increased from 0.60)

    # Side-wall fail-safe (straight sections) - REDUCED to minimize wobbling
    SIDE_LEFT_DEG = (35.0, 75.0)        # left-side sector for straight wall tracking (wider)
    SIDE_RIGHT_DEG = (-75.0, -35.0)     # right-side sector for straight wall tracking (wider)
    SIDE_NEAR_THRESHOLD = 0.55          # if side wall closer than this, nudge away (gentle correction)
    SIDE_STEER_DELTA = 10.0              # degrees to nudge steering when too close (minimal wobble)

    # Progressive wall avoidance system (3 levels: gentle → warning → PANIC)
    WARNING_WALL_DISTANCE = 0.45        # moderate correction when wall is getting close
    WARNING_SERVO_LEFT = 60.0           # moderate left turn when right wall at warning distance
    WARNING_SERVO_RIGHT = 120.0         # moderate right turn when left wall at warning distance
    
    PANIC_WALL_DISTANCE = 0.20          # HARD override when wall is dangerously close (was 0.35m, now 0.32m)
    PANIC_SERVO_HARD_LEFT = 45.0        # HARD left when right wall is too close
    PANIC_SERVO_HARD_RIGHT = 135.0      # HARD right when left wall is too close

    # Servo motion limiting
    SERVO_MAX_SLEW_DEG_PER_SEC = 120.0  # limit: how fast servo can change (deg/s)

    # Reverse maneuver timings/parameters - FIXED for proper corner handling
    # Stage 1: Pre-turn RIGHT (not left!) to position away from inside wall
    PRETURN_DURATION = 1.0              # Reduced from 1.3s - shorter pre-turn
    PRETURN_SERVO = 30                   # Turn LEFT to avoid inside wall
    PRETURN_SPEED = 400                 # Slower for control (was 500)

    # Stage 2: Reverse while following right wall
    BACKOFF_DURATION = 2.8
    BACKOFF_SERVO = 145                 # Slightly less aggressive (was 140)
    BACKOFF_SPEED = -1500
    
    # Stage 3: Go straight forward after reverse
    STRAIGHT_DURATION = 0.5
    STRAIGHT_SERVO = 90
    STRAIGHT_SPEED = 400
    
    # Stage 4: Center and resume
    CENTER_DURATION = 0.5
    CENTER_SERVO = 90

    # Wall detection parameters (from reverse_manuever.py)
    FRONT_SCAN_ANGLE_RANGE_DEG = 20     # ±10° window
    FRONT_MAX_RANGE = 1.3               # cap far depth for width calc
    # Note: WALL_WIDTH_THRESHOLD defined above at line 45 (0.08m - USER SET - DO NOT CHANGE!)

    # Reverse wall-following gain (while reversing)
    TARGET_RIGHT_DISTANCE = 0.5
    REVERSE_KP = 50.0
    
    # Emergency obstacle reverse (when stuck too close to pillar)
    EMERGENCY_TOO_CLOSE_DISTANCE = 0.10  # If obstacle closer than 15cm, trigger emergency reverse
    EMERGENCY_REVERSE_DURATION = 1.8     # Reverse for 1.8 seconds
    EMERGENCY_REVERSE_SPEED = -1000       # Reverse speed
    EMERGENCY_REVERSE_SERVO = 90         # Straight back

    def __init__(self):
        super().__init__('combined_corner_reverse_and_pillar_avoid')

        # ---------------- Vehicle params ----------------
        self.base_speed = 600
        self.min_speed = 350
        self.servo_center = 90.0
        self.servo_min = 30.0
        self.servo_max = 150.0
        self.send_rate_hz = 20.0

        # ---------------- State ----------------
        self.last_servo_deg = self.servo_center
        self.last_speed_cps = self.base_speed
        self._state_lock = threading.Lock()

        # LiDAR state
        self.last_scan_time = None
        self.last_scan_msg = None
        self.last_front_dist = float('nan')
        
        # IMU state for lap counting
        self.last_imu_msg = None
        self.current_yaw = None
        self.previous_yaw = None
        self.lap_count = 0
        self.target_laps = 3
        self.lap_completed = False
        self.initial_yaw = None
        self.yaw_initialized = False
        self.crossed_180 = False  # Track if we've crossed the 180-degree mark
        
        # ===== PERFORMANCE OPTIMIZATION: Caching =====
        self._cached_side_distances = (None, None)
        self._cached_side_distances_scan_id = None
        self._cached_lidar_obstacle = (None, None, False)
        self._cached_lidar_obstacle_scan_id = None
        self._last_log_times = {}  # For throttled logging
        self._side_smooth_prev = (None, None)  # smoothing state for side distances
        self._corner_votes = 0          # hysteresis for corner trigger
        self._corner_vote_time = 0.0    # vote window timestamp

        # Corner reverse state
        self.performing_backoff = False
        self.backoff_stage = 0  # 0 idle, 1 preturn, 2 reverse, 3 center
        self.backoff_start_time = None
        
        # Emergency obstacle reverse state
        self.performing_emergency_reverse = False
        self.emergency_reverse_start_time = None
        self.emergency_reverse_target_servo = 90  # Servo angle to use after reverse

        # Pillar avoidance state
        self.pillar_avoidance_active = False
        self.pillar_avoidance_start_time = 0.0
        self.pillar_avoid_direction = self.servo_center
        self.tracked_pillar = None  # {'color': 'red/green', 'zone': 'left/middle/right', 'timestamp': float}
        self.pillar_avoidance_cooldown = 0.0
        self.current_detections = []  # for persistence logic
        self.side_wall_disabled_until = 0.0  # Timestamp to disable side wall correction
        
        # Vision readiness flag - prevents moving before vision is ready
        self.vision_ready = False
        self.vision_ready_time = 0.0
        
        # Vision diagnostics
        self._vision_heartbeat_last_log = 0.0
        self._vision_first_frame_logged = False

        # ---------------- Vision setup ----------------
        self.pipeline = None
        self.align = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_depth_frame = None
        self._vision_stop = threading.Event()
        self._vision_thread = None
        
        # PERFORMANCE OPTIMIZATION: Background thread for frame saving (non-blocking I/O)
        self._frame_save_queue = Queue(maxsize=10)  # Limit queue size to prevent memory issues
        self._frame_saver_thread = None
        self._frame_saver_stop = threading.Event()
        # Force YOLO model loading; raise on failure
        self.get_logger().info(f"Loading YOLO model: {self.MODEL_PATH}")
        self.model = YOLO(self.MODEL_PATH)
        try:
            self.model.overrides['task'] = 'detect'
        except Exception:
            pass
        # Use Ultralytics' default device/precision selection
        self.yolo_device = 'auto'
        self.yolo_half = False
        
        # WARMUP: Run a dummy inference to compile the model (first inference is always slow!)
        self.get_logger().info("🔥 Warming up YOLO model (first inference)...")
        try:
            import numpy as np
            dummy_frame = np.zeros((240, 320, 3), dtype=np.uint8)
            _ = self.model(dummy_frame, verbose=False)
            self.get_logger().info("✅ YOLO warmup complete - model ready for fast inference!")
        except Exception as e:
            self.get_logger().warning(f"YOLO warmup failed (will warmup on first real frame): {e}")
        
        # RealSense Camera Setup
        try:
            self.get_logger().info("Initializing RealSense camera...")
            self.pipeline = rs.pipeline()
            config = rs.config()
            
            # Enable color and depth streams
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            # Start pipeline
            profile = self.pipeline.start(config)
            
            # Create align object to align depth to color
            self.align = rs.align(rs.stream.color)
            
            # Get stream info
            color_profile = profile.get_stream(rs.stream.color)
            color_intrinsics = color_profile.as_video_stream_profile().get_intrinsics()
            
            self.get_logger().info(
                f"✅ RealSense camera opened: {color_intrinsics.width}x{color_intrinsics.height} @ 30 FPS"
            )
            self.get_logger().info("🎯 Depth sensing enabled - using RealSense D-series camera")
            self.vision_enabled = True
            self._vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
            self._vision_thread.start()
            
            # Start background frame saver thread
            self._frame_saver_thread = threading.Thread(target=self._frame_saver_worker, daemon=True)
            self._frame_saver_thread.start()
            self.get_logger().info("🖼️ Background frame saver thread started")
            
        except Exception as e:
            self.get_logger().warning(f"RealSense camera init failed: {e}")
            self.get_logger().warning("Pillar avoidance disabled - no camera available")
            self.vision_enabled = False
            self.pipeline = None

        # ---------------- CAN setup ----------------
        self.bus = None
        self._can_send_lock = threading.Lock()
        try:
            self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=250000)
            self.get_logger().info("CAN bus initialized on can0 @250k")
        except Exception as e:
            self.bus = None
            self.get_logger().warning(f"CAN bus init failed: {e}")

        # ---------------- ROS subs/timers ----------------
        # Prefer '/scan' but allow remap; user can remap at runtime
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.imu_sub = self.create_subscription(Imu, '/bno055/imu', self._on_imu, 10)
        self.cmd_timer = self.create_timer(1.0 / self.send_rate_hz, self._control_loop)

        self.get_logger().info("✅ Combined controller ready: normal → pillar avoid → corner reverse")
        self.get_logger().info(f"🏁 Lap counting enabled: target = {self.target_laps} laps (IMU-based)")

    # ===================== Common helpers =====================
    def _log_throttled(self, key: str, message: str, interval: float = 1.0, level='info'):
        """Throttled logging - only log if interval seconds have passed since last log for this key"""
        now = time.time()
        last_time = self._last_log_times.get(key, 0.0)
        if now - last_time >= interval:
            self._last_log_times[key] = now
            if level == 'info':
                self.get_logger().info(message)
            elif level == 'warning':
                self.get_logger().warning(message)
            elif level == 'error':
                self.get_logger().error(message)
            return True
        return False
    
    def _set_and_send(self, speed: int, servo_deg: float):
        with self._state_lock:
            self.last_servo_deg = float(servo_deg)
            self.last_speed_cps = int(speed)
        self._send_can(speed=self.last_speed_cps, servo_deg=self.last_servo_deg)

    def _slew_servo(self, previous_servo: float, target_servo: float) -> float:
        max_step = float(self.SERVO_MAX_SLEW_DEG_PER_SEC) / float(self.send_rate_hz)
        delta = float(target_servo) - float(previous_servo)
        if delta > max_step:
            return float(previous_servo) + max_step
        if delta < -max_step:
            return float(previous_servo) - max_step
        return float(target_servo)

    def _compute_wall_correction(self, left_dist, right_dist):
        wall_correction = 0.0
        log_msg = ""
        left_close = (left_dist is not None and left_dist < self.SIDE_NEAR_THRESHOLD)
        right_close = (right_dist is not None and right_dist < self.SIDE_NEAR_THRESHOLD)
        if left_close and right_close:
            if left_dist is not None and right_dist is not None:
                if left_dist < right_dist:
                    wall_correction = self.SIDE_STEER_DELTA
                    log_msg = f"⚠️ Both walls close, left closer ({left_dist:.2f}m) → nudging right"
                else:
                    wall_correction = -self.SIDE_STEER_DELTA
                    log_msg = f"⚠️ Both walls close, right closer ({right_dist:.2f}m) → nudging left"
        elif left_close and not right_close:
            wall_correction = self.SIDE_STEER_DELTA
            log_msg = f"⚠️ Left wall too close ({left_dist:.2f}m) → steering right"
        elif right_close and not left_close:
            wall_correction = -self.SIDE_STEER_DELTA
            log_msg = f"⚠️ Right wall too close ({right_dist:.2f}m) → steering left"
        return wall_correction, log_msg

    def _frame_saver_worker(self):
        """
        OPTIMIZED: Background thread that saves frames without blocking vision loop.
        Disk I/O (cv2.imwrite) can take 10-50ms per frame - this moves it off main vision thread.
        """
        while not self._frame_saver_stop.is_set():
            try:
                # Wait for frame with timeout so we can check stop flag
                item = self._frame_save_queue.get(timeout=0.5)
                if item is None:  # Poison pill to exit
                    break
                
                annotated_frame, capture_path, num_detections = item
                
                # This is the slow I/O operation - now happens in background!
                cv2.imwrite(capture_path, annotated_frame)
                
                # Use throttled logging to avoid flooding logs
                self._log_throttled('frame_saved', f"📸 Saved annotated frame: {capture_path} ({num_detections} detections)", interval=5.0)
                
                self._frame_save_queue.task_done()
            except:
                # Queue timeout or other error - just continue
                pass
    
    def _save_annotated_capture(self, frame, frame_count: int, capture_dir: str):
        """
        OPTIMIZED: Queue frame for background saving instead of blocking vision thread.
        Annotations are done synchronously (fast), but disk I/O is deferred to background thread.
        """
        try:
            annotated_frame = frame.copy()
            try:
                with self.frame_lock:
                    detections = self.current_detections.copy() if hasattr(self, 'current_detections') else []
            except:
                detections = []
            h, w = annotated_frame.shape[:2]
            roi_y1, roi_y2 = int(h * self.ROI_Y[0]), int(h * self.ROI_Y[1])
            cv2.rectangle(annotated_frame, (0, roi_y1), (w, roi_y2), (255, 255, 0), 2)
            cv2.putText(annotated_frame, "ROI", (5, roi_y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
            left_zone_x = int(w * self.ZONE_LEFT_FRAC)
            right_zone_x = int(w * self.ZONE_MIDDLE_FRAC)
            cv2.line(annotated_frame, (left_zone_x, 0), (left_zone_x, h), (0, 255, 255), 2)
            cv2.putText(annotated_frame, "LEFT|MID", (left_zone_x - 60, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            cv2.line(annotated_frame, (right_zone_x, 0), (right_zone_x, h), (0, 255, 255), 2)
            cv2.putText(annotated_frame, "MID|RIGHT", (right_zone_x - 60, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            for idx, det in enumerate(detections):
                if 'bbox' in det:
                    x1, y1, x2, y2 = det['bbox']
                    y1 += roi_y1
                    y2 += roi_y1
                    color_bgr = (0, 0, 255) if det['color'] == 'red' else (0, 255, 0)
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color_bgr, 3)
                    zone_text = det['zone'].upper()
                    dist_text = f"{det['distance']:.2f}m"
                    label = f"{det['color'].upper()} {zone_text} {dist_text}"
                    (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                    cv2.rectangle(annotated_frame, (x1, y1 - th - 10), (x1 + tw + 10, y1), color_bgr, -1)
                    cv2.putText(annotated_frame, label, (x1 + 5, y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            info_text = f"Frame: {frame_count} | Detections: {len(detections)}"
            cv2.putText(annotated_frame, info_text, (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            capture_path = f"{capture_dir}/frame_{frame_count:06d}.jpg"
            
            # OPTIMIZED: Queue for background saving instead of blocking on cv2.imwrite
            try:
                self._frame_save_queue.put_nowait((annotated_frame, capture_path, len(detections)))
            except:
                # Queue full - skip this frame rather than blocking vision thread
                self._log_throttled('frame_queue_full', "⚠️ Frame save queue full - skipping frame", interval=5.0, level='warning')
        except Exception as e:
            self.get_logger().warning(f"Capture save error: {e}")

    # ===================== Vision utils =====================
    def _vision_loop(self):
        target_hz = 30.0
        dt = 1.0 / target_hz
        frame_count = 0
        # PERFORMANCE OPTIMIZATION: Run YOLO inference at lower rate (15Hz instead of 30Hz)
        # YOLO is expensive - we don't need 30Hz detection for obstacle avoidance
        inference_skip_frames = 2  # Process every 2nd frame = 15Hz inference
        
        # Create session-based capture folder
        session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        capture_dir = f"/home/aupp/ros2_ws/src/lidar_controller/lidar_controller/capture/session_{session_timestamp}"
        os.makedirs(capture_dir, exist_ok=True)
        self.get_logger().info(f"📁 Captures will be saved to: {capture_dir}")
        
        while not self._vision_stop.is_set() and rclpy.ok():
            frame = None
            depth_frame = None
            try:
                # Wait for RealSense frames
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
                
                # Align depth to color
                aligned_frames = self.align.process(frames)
                
                # Get aligned frames
                color_frame = aligned_frames.get_color_frame()
                aligned_depth_frame = aligned_frames.get_depth_frame()
                
                if not color_frame or not aligned_depth_frame:
                    time.sleep(0.02)
                    continue
                
                # Convert to numpy arrays
                frame = np.asanyarray(color_frame.get_data())
                depth_frame = aligned_depth_frame
                
            except Exception as e:
                self.get_logger().warning(f"RealSense read error: {e}")
                time.sleep(0.02)
                continue

            frame_count += 1
            
            # First-frame log and heartbeat
            if not self._vision_first_frame_logged:
                self.get_logger().info("✅ Vision first frame captured - camera working!")
                self._vision_first_frame_logged = True
                # Mark vision as ready after first frame processed
                self.vision_ready = True
                self.vision_ready_time = time.time()
                self.get_logger().info("🟢 Vision system READY - obstacle detection active")
            now = time.time()
            if now - self._vision_heartbeat_last_log >= 2.0:
                self.get_logger().info(f"💓 Vision heartbeat: frame #{frame_count}, camera active")
                self._vision_heartbeat_last_log = now

            # Save for potential debugging/extension
            try:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.latest_depth_frame = depth_frame
            except Exception:
                pass

            # Capture annotated debug images every 30 frames (~1 sec)
            if frame_count % 30 == 0:
                self._save_annotated_capture(frame, frame_count, capture_dir)

            # OPTIMIZED: Skip frames for YOLO inference (run at 15Hz instead of 30Hz)
            # This saves ~50% of YOLO compute time with minimal impact on detection latency
            if frame_count % inference_skip_frames == 0:
                try:
                    self._process_frame_for_pillars(frame, depth_frame)
                except Exception as e:
                    # Log ALL errors now for debugging
                    self.get_logger().error(f"❌ Vision processing error: {e}")

            time.sleep(dt)

    def _process_frame_for_pillars(self, frame, depth_frame):
        # OPTIMIZED: Extract ROI vertically (slice is a view, not a copy - very fast!)
        h, w = frame.shape[:2]
        y1, y2 = int(h * self.ROI_Y[0]), int(h * self.ROI_Y[1])
        roi_frame = frame[y1:y2, :]  # NumPy slice creates a view, not a copy
        
        # OPTIMIZED: Log only every 2 seconds instead of every frame (30Hz)
        self._log_throttled('vision_frame', f"🎬 Processing frame: {w}x{h} → ROI: {roi_frame.shape[1]}x{roi_frame.shape[0]}", interval=2.0)

        # Run inference using obstacle_simple.py method
        try:
            results = self.model(roi_frame)  # Direct call like obstacle_simple.py line 400
        except Exception as e:
            self.get_logger().error(f"❌ YOLO prediction failed: {e}")
            return
            
        r = results[0]
        boxes = r.boxes  # Direct access like obstacle_simple.py

        detected = []
        
        # Check boxes.cls like obstacle_simple.py line 426
        if boxes.cls is not None:
            xyxys = boxes.xyxy.detach().cpu().numpy() if boxes.xyxy is not None else []
            confs = boxes.conf.detach().cpu().numpy() if boxes.conf is not None else [0.0] * len(xyxys)
            class_names = r.names
            
            # OPTIMIZED: Log detections less frequently
            num_detections = len(boxes.cls)
            self._log_throttled('yolo_detections', f"🔍 YOLO inference complete: {num_detections} object(s) detected", interval=1.0)
            if num_detections == 0:
                self._log_throttled('no_objects', f"⚠️  No objects detected - check if pillar is visible and model is correct", interval=3.0)
            
            # Zone splits
            img_w = roi_frame.shape[1]
            left_end = int(img_w * self.ZONE_LEFT_FRAC)
            right_start = int(img_w * self.ZONE_MIDDLE_FRAC)

            # Ignore tiny and low-confidence boxes to reduce false activations
            min_area = (roi_frame.shape[0] * roi_frame.shape[1]) * 0.002
            for i, bb in enumerate(xyxys):
                # Skip low-confidence detections
                if confs[i] < self.CONF:
                    continue
                x1, y1b, x2, y2b = map(int, bb)
                bbox_width = x2 - x1
                bbox_height = y2b - y1b
                # Skip degenerate or tiny boxes
                if bbox_width <= 0 or bbox_height <= 0 or (bbox_width * bbox_height) < min_area:
                    continue
                patch = roi_frame[y1b:y2b, x1:x2]
                color = self._classify_red_or_green(patch)
                if color is None:
                    # OPTIMIZED: Only log classification failures occasionally
                    self._log_throttled(f'color_fail_{i}', f"⚠️  Object #{i+1}: color classification failed (not red/green)", interval=2.0)
                    continue

                center_x = (x1 + x2) // 2
                if center_x <= left_end:
                    zone = 'left'
                elif center_x >= right_start:
                    zone = 'right'
                else:
                    zone = 'middle'

                # Distance from RealSense depth (accurate per-pillar measurement!)
                # Adjust bbox coordinates to full frame (add ROI offset)
                bbox_center_x = (x1 + x2) // 2
                bbox_center_y = (y1b + y2b) // 2 + y1  # Add ROI y-offset
                
                # Get depth at bbox center
                try:
                    distance = depth_frame.get_distance(bbox_center_x, bbox_center_y)
                    
                    # If center depth is 0, try averaging a small region
                    if distance == 0.0:
                        # Average 5x5 region around center
                        depths = []
                        for dy in range(-2, 3):
                            for dx in range(-2, 3):
                                px = bbox_center_x + dx
                                py = bbox_center_y + dy
                                if 0 <= px < w and 0 <= py < h:
                                    d = depth_frame.get_distance(px, py)
                                    if d > 0:
                                        depths.append(d)
                        distance = float(np.median(depths)) if depths else 0.0
                    
                    # Fallback to LiDAR if depth unavailable
                    if distance == 0.0:
                        front_dist = self.last_front_dist
                        distance = float(front_dist) if np.isfinite(front_dist) else 1.0
                        self._log_throttled(f'lidar_fallback_{i}', f"⚠️  Using LiDAR fallback for {color.upper()} pillar #{i+1}: {distance:.2f}m", interval=2.0)
                    
                except Exception as e:
                    # Fallback to LiDAR on error
                    front_dist = self.last_front_dist
                    distance = float(front_dist) if np.isfinite(front_dist) else 1.0
                    self._log_throttled(f'depth_error_{i}', f"⚠️  Depth read error for pillar #{i+1}, using LiDAR: {e}", interval=2.0, level='warning')
                
                # More lenient distance check - only skip if clearly too far
                if distance > (self.PILLAR_DETECT_DISTANCE + 0.5):
                    # OPTIMIZED: Throttle too-far warnings
                    self._log_throttled(f'too_far_{i}', f"⚠️  {color.upper()} pillar #{i+1} too far: {distance:.2f}m > {self.PILLAR_DETECT_DISTANCE + 0.5:.1f}m", interval=2.0)
                    continue

                # OPTIMIZED: Log only when detection changes or every 0.5s
                self._log_throttled(
                    f'detection_{color}_{zone}',
                    f"👁️  Detected {color.upper()} pillar: zone={zone}, bbox={bbox_width}x{bbox_height}px, dist={distance:.2f}m (RealSense), conf={confs[i]:.2f}",
                    interval=0.5
                )

                detected.append({
                    'color': color,
                    'zone': zone,
                    'distance': distance,
                    'conf': float(confs[i]),
                    'width': bbox_width,
                    'height': bbox_height,
                    'bbox': (x1, y1b, x2, y2b),  # Store bbox coords for visualization
                })

        # Persist detections for control logic
        with self.frame_lock:
            self.current_detections = detected

        # Activate or update avoidance based on closest pillar
        # CRITICAL: Verify with LiDAR that object is actually a narrow obstacle (pillar)
        if detected:
            detected.sort(key=lambda p: p['distance'])
            closest = detected[0]
            now = time.time()
            
            # === PILLAR TRACKING: Lock onto ONE pillar, ignore others until it's passed ===
            closest = None  # Initialize to prevent errors
            if self.tracked_pillar is not None:
                # Already tracking a pillar - ONLY look for that specific pillar
                # Filter detected list to ONLY the tracked pillar (same color)
                detected = [p for p in detected if p['color'] == self.tracked_pillar['color']]
                if not detected:
                    # Lost the tracked pillar - but keep steering locked for a bit
                    elapsed = now - self.tracked_pillar['timestamp']
                    if elapsed < 2.0:  # Keep locked for 2s after losing sight
                        self.get_logger().info(f"⏳ Lost tracked {self.tracked_pillar['color']} pillar but keeping lock for {2.0-elapsed:.1f}s")
                    # Will check if passed in the zone-checking logic below
                    # No closest pillar since we lost it
                else:
                    # Found the tracked pillar - update timestamp ONLY, servo stays LOCKED
                    closest = min(detected, key=lambda x: x['distance'])
                    self.tracked_pillar['timestamp'] = now
                    self.get_logger().info(
                        f"🔒 Tracking {self.tracked_pillar['color']} pillar: zone={closest['zone']} dist={closest['distance']:.2f}m "
                        f"| Servo LOCKED at {self.pillar_avoid_direction:.1f}° (NO UPDATES)"
                    )
            else:
                # Not tracking any pillar yet - can detect new one
                if detected:
                    closest = min(detected, key=lambda x: x['distance'])
                else:
                    closest = None
            
            # If no closest pillar found AND not already tracking, skip activation logic
            if closest is None and not self.pillar_avoidance_active:
                return
            
            # If we're tracking but lost sight, continue to check if pillar passed (don't return early)
            
            # Only process activation logic if we have a valid closest pillar
            if closest is not None:
                # LIDAR VERIFICATION: Check if detected object is actually a pillar (narrow obstacle)
                # This prevents premature avoidance when pillar is still far or when seeing a wall
                lidar_dist, lidar_width, lidar_is_obstacle = self._detect_lidar_obstacle()
                
                # More lenient LiDAR confirmation - allow avoidance if:
                # 1. LiDAR confirms pillar OR
                # 2. Vision is very confident AND (LiDAR not ready OR pillar is close in vision)
                # 3. For FIRST obstacle detection, be more lenient
                lidar_confirms_pillar = (
                    lidar_is_obstacle and 
                    lidar_dist is not None and 
                    lidar_dist <= self.OBSTACLE_DETECT_DISTANCE and
                    lidar_width is not None and
                    lidar_width < self.WALL_WIDTH_THRESHOLD
                )
                
                # Lenient mode for first obstacle or when LiDAR might not be ready
                vision_very_confident = (closest['conf'] >= 0.80 and closest['distance'] <= 1.5)
                lidar_not_contradicting = (lidar_dist is None or lidar_width is None or not np.isfinite(self.last_front_dist))
                first_obstacle_mode = (not self.pillar_avoidance_active and now > self.pillar_avoidance_cooldown)
                
                # Allow activation if LiDAR confirms OR (high confidence vision + LiDAR not ready + first obstacle)
                can_activate = lidar_confirms_pillar or (vision_very_confident and lidar_not_contradicting and first_obstacle_mode)
                
                if not can_activate and not self.pillar_avoidance_active:
                    # Vision sees pillar but verification pending
                    width_str = f"{lidar_width:.3f}m" if lidar_width else 'N/A'
                    dist_str = f"{lidar_dist:.3f}m" if lidar_dist else 'N/A'
                    self.get_logger().info(
                        f"⏳ Vision detected {closest['color']} pillar at {closest['distance']:.2f}m (conf={closest['conf']:.2f}), "
                        f"LiDAR verification: dist={dist_str}, width={width_str}, can_activate={can_activate}"
                    )
                elif not can_activate and self.pillar_avoidance_active:
                    # Already avoiding but LiDAR no longer sees obstacle - might have passed it
                    self.get_logger().info(
                        f"✅ LiDAR no longer detects narrow obstacle (pillar passed?) - continuing avoidance until zone target reached"
                    )

                # RED avoidance: Turn RIGHT to move pillar to LEFT side of ROI
                # Only activate if red is in MIDDLE or RIGHT zones (needs to be moved left)
                if closest['color'] == 'red' and closest['zone'] in ('middle', 'right') and not self.pillar_avoidance_active and can_activate:
                    # First activation ONLY - LOCK onto this pillar
                    self.pillar_avoidance_active = True
                    self.pillar_avoidance_start_time = now
                    self.side_wall_disabled_until = now + 1.0  # Disable side wall for 1.0s (allow wall help sooner)
                    self.tracked_pillar = {'color': 'red', 'zone': closest['zone'], 'timestamp': now}
                    servo = self._pillar_avoidance_servo([closest])
                    if servo is not None:
                        self.pillar_avoid_direction = float(servo)
                    
                    # Determine actual direction based on zone
                    if closest['zone'] == 'right':
                        direction_msg = "STRONG RIGHT ➡️ to move pillar LEFT"
                    else:  # middle zone
                        direction_msg = "MODERATE RIGHT ➡️ to move pillar LEFT"
                    
                    self.get_logger().info(
                        f"🔴 RED pillar avoidance ON: zone={closest['zone']} dist={closest['distance']:.2f}m "
                        f"→ {direction_msg} servo={self.pillar_avoid_direction:.1f}° (target: LEFT zone) [🔒 LOCKED - WILL NOT UPDATE]"
                    )
                        
                # GREEN avoidance: Turn LEFT to move pillar to RIGHT side of ROI
                # Only activate if green is in MIDDLE or LEFT zones (needs to be moved right)
                elif closest['color'] == 'green' and closest['zone'] in ('middle', 'left') and not self.pillar_avoidance_active and can_activate:
                    # First activation ONLY - LOCK onto this pillar
                    self.pillar_avoidance_active = True
                    self.pillar_avoidance_start_time = now
                    self.side_wall_disabled_until = now + 1.0  # Disable side wall for 1.0s (allow wall help sooner)
                    self.tracked_pillar = {'color': 'green', 'zone': closest['zone'], 'timestamp': now}
                    servo = self._pillar_avoidance_servo([closest])
                    if servo is not None:
                        self.pillar_avoid_direction = float(servo)
                    
                    # Determine actual direction based on zone
                    if closest['zone'] == 'left':
                        direction_msg = "STRONG LEFT ⬅️ to move pillar RIGHT"
                    else:  # middle zone
                        direction_msg = "MODERATE LEFT ⬅️ to move pillar RIGHT"
                    
                    self.get_logger().info(
                        f"🟢 GREEN pillar avoidance ON: zone={closest['zone']} dist={closest['distance']:.2f}m "
                        f"→ {direction_msg} servo={self.pillar_avoid_direction:.1f}° (target: RIGHT zone) [🔒 LOCKED - WILL NOT UPDATE]"
                    )
                        
                # ALWAYS activate for first obstacle, regardless of zone (fixes first obstacle issue)
                elif not self.pillar_avoidance_active and now > self.pillar_avoidance_cooldown and can_activate:
                    self.pillar_avoidance_active = True
                    self.pillar_avoidance_start_time = now
                    self.side_wall_disabled_until = now + 1.0  # Disable side wall for 1.0s (allow wall help sooner)
                    self.tracked_pillar = {'color': closest['color'], 'zone': closest['zone'], 'timestamp': now}
                    servo = self._pillar_avoidance_servo(detected)
                    if servo is not None:
                        self.pillar_avoid_direction = float(servo)
                    direction = "LEFT ⬅️" if servo < 90 else "RIGHT ➡️" if servo > 90 else "CENTER"
                    self.get_logger().info(
                        f"🎯 Pillar avoidance ON (first detection, LiDAR verified: width={lidar_width:.3f}m < {self.WALL_WIDTH_THRESHOLD:.3f}m): "
                        f"closest={closest['color']} zone={closest['zone']} dist={closest['distance']:.2f}m → "
                        f"{direction} servo={self.pillar_avoid_direction:.1f}° [side wall disabled for 1.0s]"
                    )

            # Check if we've successfully passed the pillar
            # Success conditions:
            # - RED pillar: reached LEFT zone OR no longer detected
            # - GREEN pillar: reached RIGHT zone OR no longer detected
            red_in_target = any(d['color'] == 'red' and d['zone'] == 'left' for d in detected)
            green_in_target = any(d['color'] == 'green' and d['zone'] == 'right' for d in detected)
            red_still_needs_avoiding = any(d['color'] == 'red' and d['zone'] in ('middle', 'right') for d in detected)
            green_still_needs_avoiding = any(d['color'] == 'green' and d['zone'] in ('middle', 'left') for d in detected)
            
            if self.pillar_avoidance_active:
                elapsed = time.time() - self.pillar_avoidance_start_time
                
                # Check if pillar has been successfully passed using BOTH vision AND lidar
                lidar_dist, lidar_width, lidar_is_obstacle = self._detect_lidar_obstacle()
                lidar_clear = not lidar_is_obstacle or lidar_dist is None or lidar_dist > 1.0
                
                # Success: pillar in target zone OR (no longer detected visually AND lidar clear)
                pillar_passed = False
                if red_in_target or green_in_target:
                    pillar_passed = True
                    self.get_logger().info("✅ Pillar reached target zone (vision confirms)")
                elif not red_still_needs_avoiding and not green_still_needs_avoiding and elapsed > 0.5:
                    if lidar_clear:
                        pillar_passed = True
                        self.get_logger().info("✅ Pillar passed (vision + lidar confirm clear)")
                    else:
                        self.get_logger().info(f"⏳ Vision lost pillar but lidar still sees obstacle at {lidar_dist:.2f}m - continuing avoidance")
                
                if pillar_passed:
                    self.pillar_avoidance_active = False
                    self.pillar_avoid_direction = self.servo_center
                    self.tracked_pillar = None  # Clear tracked pillar
                    self.pillar_avoidance_cooldown = time.time() + 1.0
                    self.get_logger().info("✅ Pillar avoidance OFF - cooldown 1.0s before detecting next pillar")
                elif red_still_needs_avoiding or green_still_needs_avoiding:
                    # Still need to avoid - refresh timer
                    self.pillar_avoidance_start_time = time.time()
        else:
            # No detections recently: let avoidance finish naturally via cooldown/timeout in control loop
            pass

    def _classify_red_or_green(self, bgr_patch):
        """
        OPTIMIZED: Color classification using efficient NumPy operations.
        - Use int32 for sums to avoid overflow (faster than float32)
        - Compute only necessary intermediate values
        - Avoid unnecessary array copies
        """
        if bgr_patch is None or bgr_patch.size == 0:
            return None
        
        # OPTIMIZED: Work with uint8 directly, sum in int32 (faster than float conversion)
        # Cast to int32 only for sum to avoid overflow, then do division
        b = bgr_patch[:, :, 0].astype(np.int32)
        g = bgr_patch[:, :, 1].astype(np.int32)
        r = bgr_patch[:, :, 2].astype(np.int32)
        
        # Sum with small epsilon to avoid division by zero
        s = b + g + r + 1  # Add 1 instead of 1e-6 (integer math is faster)
        
        # Normalize to 0-1 range (do division once)
        b_norm = b / s
        g_norm = g / s
        r_norm = r / s
        
        # Calculate masks (boolean operations are fast)
        red_mask = (r_norm > 0.40) & ((r_norm - g_norm) > 0.08) & (r_norm > b_norm)
        green_mask = (g_norm > 0.40) & ((g_norm - r_norm) > 0.08) & (g_norm > b_norm)
        
        # Use np.count_nonzero which is faster than .mean() for boolean arrays
        red_score = np.count_nonzero(red_mask)
        green_score = np.count_nonzero(green_mask)
        
        return 'red' if red_score > green_score else 'green'

    def _pillar_avoidance_servo(self, pillars):
        """
        Persistent zone-based avoidance with WALL SAFETY:
        - Green pillar → turn LEFT (lower servo values 30-70°) - MORE AGGRESSIVE
        - Red pillar → turn RIGHT (higher servo values 110-150°) - MORE AGGRESSIVE
        - Adjusted for safety when near walls
        """
        if not pillars:
            return None
        pillars.sort(key=lambda p: p['distance'])
        p = pillars[0]
        color, zone, dist = p['color'], p['zone'], p['distance']
        close, med = 0.45, 0.55
        
        # Get wall distances for safety checks
        left_dist, right_dist = self._get_side_distances()
        left_wall_close = (left_dist is not None and left_dist < 0.45)  # Inside wall danger zone
        right_wall_close = (right_dist is not None and right_dist < 0.45)  # Outside wall danger zone
        
        if color == 'green':
            # GREEN = TURN LEFT (lower servo angles) - MORE AGGRESSIVE
            if zone == 'middle':
                # VERY aggressive left turn in middle zone
                return 30 if dist <= close else 38 if dist <= med else 48
            elif zone == 'right':
                # Moderate left when already in right - almost done
                return 48 if dist <= close else 58 if dist <= med else 65
            else:  # left zone
                # Left zone: pillar is near LEFT wall - use GENTLER turn to avoid hitting wall
                if left_wall_close:
                    # SAFETY: Left wall too close - use gentle turn to avoid hitting corner
                    return 40 if dist <= close else 45 if dist <= med else 50
                else:
                    # Normal: VERY aggressive left turn to move pillar across
                    return 30 if dist <= close else 35 if dist <= med else 40
        
        elif color == 'red':
            # RED = TURN RIGHT (higher servo angles) - MORE AGGRESSIVE
            if zone == 'middle':
                # VERY aggressive right turn in middle zone
                return 150 if dist <= close else 142 if dist <= med else 132
            elif zone == 'left':
                # Moderate right when already in left - almost done
                return 132 if dist <= close else 122 if dist <= med else 115
            else:  # right zone
                # Right zone: pillar is near RIGHT wall - use GENTLER turn to avoid hitting wall
                if right_wall_close:
                    # SAFETY: Right wall too close - use gentle turn to avoid hitting corner
                    return 140 if dist <= close else 135 if dist <= med else 130
                else:
                    # Normal: VERY aggressive right turn to move pillar across
                    return 150 if dist <= close else 145 if dist <= med else 140
        
        return None

    # ===================== LiDAR utils =====================
    def _on_scan(self, msg: LaserScan):
        self.last_scan_msg = msg
        self.last_scan_time = self.get_clock().now()

        # Compute front sector robust distance
        degs = self._build_signed_deg_array(msg)
        rngs = np.array(msg.ranges, dtype=np.float32)
        m_front = self._sector_mask_signed(degs, *self.FRONT_DEG)
        self.last_front_dist = self._robust_sector_distance(rngs, m_front)

        # Optionally could precompute side distances if needed later
    
    # ===================== IMU utils =====================
    def _quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def _on_imu(self, msg: Imu):
        """Process IMU data for lap counting"""
        self.last_imu_msg = msg
        
        # Convert quaternion to yaw angle (in degrees)
        orientation_q = msg.orientation
        
        try:
            (roll, pitch, yaw) = self._quaternion_to_euler(
                orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
            )
            # Convert yaw from radians to degrees and normalize to 0-360
            yaw_deg = math.degrees(yaw)
            if yaw_deg < 0:
                yaw_deg += 360.0
            
            self.previous_yaw = self.current_yaw
            self.current_yaw = yaw_deg
            
            # Initialize reference yaw on first reading
            if not self.yaw_initialized:
                self.initial_yaw = yaw_deg
                self.yaw_initialized = True
                self.get_logger().info(f"IMU initialized: starting yaw = {yaw_deg:.1f} degrees")
                return
            
            # Lap counting logic: detect when robot crosses start line (initial_yaw)
            if self.previous_yaw is not None and self.lap_count < self.target_laps:
                # Define a crossing window around initial yaw (e.g., ±10 degrees)
                crossing_window = 15.0
                
                # Check if we've crossed 180 degrees from start (halfway point)
                halfway_angle = (self.initial_yaw + 180.0) % 360.0
                
                # Detect crossing the halfway point (prevents premature lap counting)
                if not self.crossed_180:
                    # Check if we're near the halfway point
                    angle_diff_halfway = abs(self.current_yaw - halfway_angle)
                    if angle_diff_halfway < 180:
                        angle_diff_halfway = min(angle_diff_halfway, 360 - angle_diff_halfway)
                    
                    if angle_diff_halfway < crossing_window:
                        self.crossed_180 = True
                        self.get_logger().info(f"Halfway point reached: yaw = {self.current_yaw:.1f} degrees")
                
                # Detect crossing the start line (only after crossing halfway)
                if self.crossed_180:
                    angle_diff_start = abs(self.current_yaw - self.initial_yaw)
                    if angle_diff_start > 180:
                        angle_diff_start = 360 - angle_diff_start
                    
                    if angle_diff_start < crossing_window:
                        # We're back at the start line - lap completed!
                        if not self.lap_completed:
                            self.lap_count += 1
                            self.lap_completed = True
                            self.crossed_180 = False
                            self.get_logger().info(
                                f"LAP {self.lap_count} COMPLETED! "
                                f"(yaw: {self.current_yaw:.1f} degrees, target: {self.target_laps} laps)"
                            )
                            
                            if self.lap_count >= self.target_laps:
                                self.get_logger().info(
                                    f"TARGET REACHED! {self.lap_count} laps completed - "
                                    f"robot will stop on next control cycle"
                                )
                    else:
                        # Reset lap completion flag when we're away from start line
                        self.lap_completed = False
            
        except Exception as e:
            self.get_logger().warning(f"IMU processing error: {e}")

    def _build_signed_deg_array(self, msg):
        n = len(msg.ranges)
        ang_rads = msg.angle_min + np.arange(n) * msg.angle_increment
        return np.degrees(ang_rads)

    def _sector_mask_signed(self, deg_arr, lo, hi):
        if lo <= hi:
            return (deg_arr >= lo) & (deg_arr <= hi)
        return (deg_arr >= lo) | (deg_arr <= hi)

    def _robust_sector_distance(self, ranges, mask):
        vals = np.asarray(ranges)[mask]
        vals = vals[np.isfinite(vals)]
        vals = vals[(vals >= self.RANGE_MIN) & (vals <= self.RANGE_MAX)]
        if vals.size < self.MIN_HITS:
            return float('nan')
        vals.sort()
        k = max(5, int(self.K_FRACT * vals.size))
        k = min(k, vals.size)
        return float(np.mean(vals[:k]))

    def _get_right_wall_distance(self):
        msg = self.last_scan_msg
        if msg is None:
            return None
        ranges = np.array(msg.ranges)
        angle_increment = msg.angle_increment
        total_points = len(ranges)
        # Assuming -90°..+90° FOV; right ~ 70°..90°
        right_start_rad = np.radians(70)
        right_end_rad = np.radians(90)
        right_start_idx = int((right_start_rad + np.pi/2) / angle_increment)
        right_end_idx = int((right_end_rad + np.pi/2) / angle_increment)
        right_start_idx = max(0, min(total_points - 1, right_start_idx))
        right_end_idx = max(0, min(total_points, right_end_idx))
        right_ranges = ranges[right_start_idx:right_end_idx]
        valid = right_ranges[np.isfinite(right_ranges)]
        valid = valid[(valid > 0) & (valid < 2.0)]
        if valid.size > 0:
            return float(np.median(valid))
        return None

    def _get_front_width_and_distance(self, angle_deg=15.0, max_range=None):
        msg = self.last_scan_msg
        if msg is None:
            return None, None, False

        ranges = np.array(msg.ranges)
        angle_increment = msg.angle_increment
        total_points = len(ranges)
        
        if max_range is None:
            max_range = self.RANGE_MAX

        # Calculate window indices
        front_angle_rad = np.radians(angle_deg)
        center_index = total_points // 2
        angle_range_indices = int(front_angle_rad / angle_increment)
        start_idx = max(0, center_index - angle_range_indices)
        end_idx = min(total_points, center_index + angle_range_indices)

        front_ranges = ranges[start_idx:end_idx]
        valid_mask = np.isfinite(front_ranges) & (front_ranges > 0) & (front_ranges < max_range)
        valid_front = front_ranges[valid_mask]
        
        if valid_front.size == 0:
            return None, None, False
        
        front_distance = float(np.min(valid_front))

        # Width calculation: count consecutive points near the minimum distance
        # Adaptive tolerance improves robustness across distances
        distance_tolerance = max(0.05, 0.12 * front_distance)
        consecutive = 0
        max_consecutive = 0
        
        for i in range(start_idx, end_idx):
            if i < len(ranges) and np.isfinite(ranges[i]) and abs(ranges[i] - front_distance) <= distance_tolerance:
                consecutive += 1
                if consecutive > max_consecutive:
                    max_consecutive = consecutive
            else:
                consecutive = 0
        
        if max_consecutive > 0:
            angle_span = max_consecutive * angle_increment
            est_width = front_distance * angle_span
            is_obstacle = est_width < self.WALL_WIDTH_THRESHOLD
            
            # OPTIMIZED: Log only when obstacle type changes or every 1s (was logging 20Hz!)
            obstacle_type = 'OBSTACLE (narrow)' if is_obstacle else 'WALL (wide)'
            self._log_throttled(
                f'lidar_width_{obstacle_type}',
                f"📏 LiDAR width: dist={front_distance:.3f}m, pts={max_consecutive}, "
                f"span={np.degrees(angle_span):.1f}°, width={est_width:.3f}m, "
                f"thresh={self.WALL_WIDTH_THRESHOLD:.3f}m → {obstacle_type}",
                interval=1.0
            )
            
            return front_distance, est_width, is_obstacle
        
        return front_distance, None, False

    def _front_distance_and_is_wall(self):
        """Corner detection: uses narrow angle (10°) and shorter range (1.3m)"""
        dist, width, is_obstacle = self._get_front_width_and_distance(
            angle_deg=self.FRONT_SCAN_ANGLE_RANGE_DEG / 2.0,  # ±10°
            max_range=self.FRONT_MAX_RANGE  # 1.3m
        )
        if dist is None:
            return None, False
        is_wall = not is_obstacle  # wall = NOT obstacle (width >= threshold)
        return dist, is_wall

    def _get_obstacle_width_and_distance(self):
        """Obstacle detection: uses wider angle (15°) and full range (4.0m)"""
        return self._get_front_width_and_distance(
            angle_deg=15.0,  # ±15° (matching FRONT_DEG)
            max_range=self.RANGE_MAX  # 4.0m
        )

    def _is_corner(self):
        """
        Detect corner by checking if front obstacle is a WALL (not narrow obstacle).
        Uses width measurement to distinguish walls from pillars.
        Enhanced to trigger earlier when red pillar is near outer wall.
        CRITICAL: Must verify it's actually a WALL (width > 0.08m) not a pillar!
        """
        front_dist, is_wall = self._front_distance_and_is_wall()
        if front_dist is None:
            # Log why we can't detect corner
            self.get_logger().info("🚫 Corner check: front distance is None (no LiDAR data)")
            return False
        
        # CRITICAL: Must be a wall (not narrow obstacle/pillar)
        # This prevents false corner detection when approaching a pillar
        if not is_wall:
            self.get_logger().info(
                f"🚫 Not corner - narrow object detected (pillar?): front={front_dist:.3f}m, "
                f"width < {self.WALL_WIDTH_THRESHOLD:.3f}m"
            )
            return False
        
        # Check if we have red pillar detected in right zone (near outer wall)
        # This indicates exit obstacle that might interfere with corner detection
        has_red_in_right_zone = False
        try:
            with self.frame_lock:
                detections = self.current_detections.copy() if hasattr(self, 'current_detections') else []
            has_red_in_right_zone = any(
                d['color'] == 'red' and d['zone'] == 'right' and d['distance'] <= 1.5
                for d in detections
            )
        except:
            pass
        
        # Use more aggressive corner detection window if red pillar is near outer wall
        if has_red_in_right_zone:
            # Expand the window to trigger earlier (0.25 → 0.45m)
            in_window = (self.FRONT_CORNER_WINDOW[0] <= front_dist <= 0.45)
            if not in_window:
                return False
            
            right_dist = self._get_right_wall_distance()
            if right_dist is None:
                return False
            
            # More lenient right wall threshold when red pillar present
            is_corner = right_dist <= 0.70  # Increased from 0.60
            if is_corner:
                self.get_logger().info(
                    f"🔴 CORNER DETECTED (red pillar near outer wall): "
                    f"front={front_dist:.3f}m (WALL verified), right={right_dist:.3f}m → TRIGGERING REVERSE EARLY"
                )
            return is_corner
        else:
            # Normal corner detection
            in_window = (self.FRONT_CORNER_WINDOW[0] <= front_dist <= self.FRONT_CORNER_WINDOW[1])
            if not in_window:
                # Log when front wall is detected but outside corner window
                if front_dist < self.FRONT_CORNER_WINDOW[0]:
                    self.get_logger().info(
                        f"⚠️ Front WALL too close: {front_dist:.3f}m < {self.FRONT_CORNER_WINDOW[0]:.2f}m "
                        f"(already past corner window!)"
                    )
                elif front_dist > self.FRONT_CORNER_WINDOW[1]:
                    # Don't log - too far away, normal operation
                    pass
                return False
            
            right_dist = self._get_right_wall_distance()
            if right_dist is None:
                self.get_logger().info(
                    f"⚠️ Front wall in range ({front_dist:.3f}m) but no right wall detected"
                )
                return False
            
            # Corner: front wall in window and right wall is near
            is_corner = right_dist <= self.RIGHT_NEAR_THRESHOLD
            if is_corner:
                self.get_logger().info(
                    f"🏁 CORNER DETECTED: front={front_dist:.3f}m (WALL verified), "
                    f"right={right_dist:.3f}m ≤ {self.RIGHT_NEAR_THRESHOLD:.2f}m → TRIGGERING REVERSE"
                )
            else:
                self.get_logger().info(
                    f"🚧 Front wall detected ({front_dist:.3f}m) but right wall too far: "
                    f"right={right_dist:.3f}m > {self.RIGHT_NEAR_THRESHOLD:.2f}m (not a corner)"
                )
            return is_corner
    
    def _is_approaching_corner(self):
        """
        Detect if we're approaching a corner (but not close enough to trigger yet).
        This is used to disable side wall corrections that interfere with corner approach.
        Uses a wider detection window than _is_corner() to catch early approach.
        """
        front_dist, is_wall = self._front_distance_and_is_wall()
        if front_dist is None:
            return False
        
        # Must be a wall (not pillar)
        if not is_wall:
            return False
        
        # Check if front wall is within approach window (wider than corner trigger)
        # Corner triggers at 0.25-0.35m (or 0.45m with red pillar)
        # Approach detection starts at 0.5m to disable side corrections early
        in_approach_window = (0.35 < front_dist <= 0.60)
        
        if not in_approach_window:
            return False
        
        # Check if right wall is reasonably close (likely heading to corner)
        right_dist = self._get_right_wall_distance()
        if right_dist is None:
            return False
        
        # More lenient threshold - just checking if we're in a corner-like configuration
        approaching = right_dist <= 0.80  # More lenient than corner threshold (0.60)
        
        if approaching:
            self.get_logger().info(
                f"🚦 Approaching corner detected: front={front_dist:.3f}m (WALL), "
                f"right={right_dist:.3f}m → DISABLING side wall corrections"
            )
        
        return approaching
    
    def _detect_lidar_obstacle(self):
        """
        OPTIMIZED: Detect narrow obstacles (pillars) in front using LiDAR width measurement.
        Returns: (distance, width, is_narrow_obstacle) or (None, None, False)
        Uses 0.08m threshold - objects narrower than this are pillars, wider are walls.
        CACHED per scan to avoid recalculating multiple times per control loop.
        """
        msg = self.last_scan_msg
        if msg is None:
            return None, None, False
        
        # Use scan ID as cache key
        scan_id = id(msg)
        
        # Return cached value if same scan
        if scan_id == self._cached_lidar_obstacle_scan_id:
            return self._cached_lidar_obstacle
        
        # Calculate and cache
        dist, width, is_obstacle = self._get_obstacle_width_and_distance()
        
        if dist is None or width is None:
            result = (None, None, False)
            self._cached_lidar_obstacle = result
            self._cached_lidar_obstacle_scan_id = scan_id
            return result
        
        # Only return True if it's close enough and narrow (obstacle/pillar, not wall)
        if dist <= self.OBSTACLE_DETECT_DISTANCE and is_obstacle:
            # OPTIMIZED: Throttle confirmation logging
            self._log_throttled(
                'lidar_pillar',
                f"✅ LiDAR PILLAR CONFIRMED: dist={dist:.3f}m (≤{self.OBSTACLE_DETECT_DISTANCE:.2f}m), "
                f"width={width:.3f}m (< {self.WALL_WIDTH_THRESHOLD:.3f}m) → NARROW OBSTACLE",
                interval=0.5
            )
            result = (dist, width, True)
        elif dist <= self.OBSTACLE_DETECT_DISTANCE and not is_obstacle:
            # OPTIMIZED: Throttle wall detection logging
            self._log_throttled(
                'lidar_wall',
                f"🧱 LiDAR detects WALL: dist={dist:.3f}m, width={width:.3f}m (≥ {self.WALL_WIDTH_THRESHOLD:.3f}m) → NOT a pillar",
                interval=0.5
            )
            result = (dist, width, False)
        else:
            result = (dist, width, False)
        
        # Cache the result
        self._cached_lidar_obstacle = result
        self._cached_lidar_obstacle_scan_id = scan_id
        
        return result

    # ===================== Control loop =====================
    def _control_loop(self):
        # PRIORITY 0: Check if target laps completed - STOP ROBOT
        if self.lap_count >= self.target_laps:
            self.get_logger().info(
                f"RACE COMPLETE! {self.lap_count}/{self.target_laps} laps finished - STOPPING ROBOT"
            )
            self.stop_robot()
            return
        
        # Log lap progress periodically
        if self.yaw_initialized and self.current_yaw is not None:
            yaw_from_start = abs(self.current_yaw - self.initial_yaw) if self.initial_yaw else 0
            if yaw_from_start > 180:
                yaw_from_start = 360 - yaw_from_start
            self._log_throttled(
                'lap_status',
                f"🏁 Lap progress: {self.lap_count}/{self.target_laps} laps | "
                f"Current yaw: {self.current_yaw:.1f}° | Distance from start: {yaw_from_start:.1f}°",
                interval=3.0
            )
        
        # If performing corner backoff, run it and return
        if self.performing_backoff:
            self._handle_backoff()
            return
        
        # If performing emergency reverse, run it and return
        if self.performing_emergency_reverse:
            self._handle_emergency_reverse()
            return

        # EMERGENCY CHECK: Too close to obstacle? Reverse!
        # Check if narrow obstacle (pillar) is dangerously close
        lidar_dist, lidar_width, lidar_is_obstacle = self._detect_lidar_obstacle()
        if (lidar_is_obstacle and 
            lidar_dist is not None and 
            lidar_dist < self.EMERGENCY_TOO_CLOSE_DISTANCE and
            lidar_width is not None and 
            lidar_width < self.WALL_WIDTH_THRESHOLD):
            
            # TOO CLOSE! Start emergency reverse
            self.get_logger().warning(
                f"🚨 EMERGENCY! Obstacle too close: {lidar_dist:.3f}m < {self.EMERGENCY_TOO_CLOSE_DISTANCE:.2f}m "
                f"(width={lidar_width:.3f}m) → REVERSING!"
            )
            self._start_emergency_reverse()
            return

        # PRIORITY 1: Corner detection (highest priority - must check FIRST)
        # Corner detection should override ALL other logic including pillar avoidance
        # Check if it's actually a WALL (not narrow obstacle) to avoid false positives
        is_corner_now = self._is_corner()
        # hysteresis — require 2 positive checks within 0.4s
        now_ts = time.time()
        if now_ts - self._corner_vote_time > 0.4:
            self._corner_votes = 0
        self._corner_vote_time = now_ts
        if is_corner_now:
            self._corner_votes += 1

        if self._corner_votes >= 2:
            self._corner_votes = 0
            # Immediately disable any active pillar avoidance when corner is detected
            if self.pillar_avoidance_active:
                self.get_logger().info("🛑 Disabling pillar avoidance - CORNER takes priority!")
                self.pillar_avoidance_active = False
                self.pillar_avoid_direction = self.servo_center
                self.tracked_pillar = None  # Clear tracked pillar
            self._start_backoff()
            return

        # PRE-CHECK: Check if approaching corner (to disable side wall corrections)
        # This prevents side wall steering from interfering with corner approach
        approaching_corner = self._is_approaching_corner()
        
        # Get wall distances for progressive safety checks
        left_dist, right_dist = self._get_side_distances()
        
        # PRIORITY 2: Emergency side-wall PANIC override (HARDEST correction)
        # If a side wall is dangerously close, steer hard away and slow down.
        # BUT: Don't trigger panic if we're approaching a corner (corner has priority)
        left_panic = (left_dist is not None and left_dist < self.PANIC_WALL_DISTANCE)
        right_panic = (right_dist is not None and right_dist < self.PANIC_WALL_DISTANCE)
        
        # Don't panic if approaching corner - let corner detection handle it
        if (left_panic or right_panic) and not approaching_corner:
            # CRITICAL: Disable pillar avoidance - wall safety takes priority!
            if self.pillar_avoidance_active:
                self.get_logger().warning("🛑 PANIC MODE - Disabling pillar avoidance, WALL SAFETY PRIORITY!")
                self.pillar_avoidance_active = False
                self.pillar_avoid_direction = self.servo_center
            
            # Decide steering: if both panic, steer away from the closer wall
            if left_panic and right_panic:
                if left_dist is not None and right_dist is not None:
                    steer_servo = self.PANIC_SERVO_HARD_RIGHT if left_dist <= right_dist else self.PANIC_SERVO_HARD_LEFT
                else:
                    steer_servo = self.PANIC_SERVO_HARD_LEFT  # fallback
            elif left_panic:
                steer_servo = self.PANIC_SERVO_HARD_RIGHT
            else:  # right_panic
                steer_servo = self.PANIC_SERVO_HARD_LEFT

            panic_speed = int(max(self.min_speed, self.base_speed * 0.5))  # Increased from 0.4 to 0.5 for better control
            steer_servo = max(self.servo_min, min(self.servo_max, steer_servo))

            # Log and send immediately, then return to avoid other logic overriding
            if left_panic and not right_panic:
                self.get_logger().warning(f"🚨 PANIC LEFT! {left_dist:.2f}m < {self.PANIC_WALL_DISTANCE:.2f}m - HARD RIGHT {steer_servo:.0f}°, speed {panic_speed}")
            elif right_panic and not left_panic:
                self.get_logger().warning(f"🚨 PANIC RIGHT! {right_dist:.2f}m < {self.PANIC_WALL_DISTANCE:.2f}m - HARD LEFT {steer_servo:.0f}°, speed {panic_speed}")
            else:
                self.get_logger().warning(
                    f"🚨 PANIC BOTH! L:{(left_dist if left_dist is not None else float('nan')):.2f}m "
                    f"R:{(right_dist if right_dist is not None else float('nan')):.2f}m - servo {steer_servo:.0f}°, speed {panic_speed}"
                )

            self._set_and_send(speed=panic_speed, servo_deg=steer_servo)
            return
        elif (left_panic or right_panic) and approaching_corner:
            # Would panic but approaching corner - log and let corner logic take over
            left_str = f"{left_dist:.2f}m" if left_dist else 'N/A'
            right_str = f"{right_dist:.2f}m" if right_dist else 'N/A'
            self.get_logger().info(
                f"⚠️ Wall panic condition detected BUT approaching corner - "
                f"letting corner detection take priority (L={left_str}, R={right_str})"
            )
        
        # PRIORITY 3: WARNING level wall avoidance (moderate correction before panic)
        # Triggers when wall is close but not yet at panic distance
        # This provides earlier intervention, especially during pillar avoidance
        left_warning = (left_dist is not None and self.PANIC_WALL_DISTANCE < left_dist < self.WARNING_WALL_DISTANCE)
        right_warning = (right_dist is not None and self.PANIC_WALL_DISTANCE < right_dist < self.WARNING_WALL_DISTANCE)
        
        if (left_warning or right_warning) and not approaching_corner and not self.pillar_avoidance_active:
            # Apply moderate steering correction - this CAN override pillar avoidance
            if left_warning and right_warning:
                # Both walls at warning - steer away from closer one
                if left_dist is not None and right_dist is not None:
                    steer_servo = self.WARNING_SERVO_RIGHT if left_dist <= right_dist else self.WARNING_SERVO_LEFT
                else:
                    steer_servo = self.servo_center  # fallback
            elif left_warning:
                steer_servo = self.WARNING_SERVO_RIGHT  # steer right away from left wall
            else:  # right_warning
                steer_servo = self.WARNING_SERVO_LEFT   # steer left away from right wall
            
            warning_speed = int(self.base_speed * 0.8)  # Slow down a bit
            steer_servo = max(self.servo_min, min(self.servo_max, steer_servo))
            
            # Log warning
            if left_warning and not right_warning:
                self.get_logger().warning(f"⚠️ WARNING LEFT! {left_dist:.2f}m - moderate RIGHT {steer_servo:.0f}°")
            elif right_warning and not left_warning:
                self.get_logger().warning(f"⚠️ WARNING RIGHT! {right_dist:.2f}m - moderate LEFT {steer_servo:.0f}°")
            else:
                self.get_logger().warning(f"⚠️ WARNING BOTH! L:{left_dist:.2f}m R:{right_dist:.2f}m - servo {steer_servo:.0f}°")
            
            self._set_and_send(speed=warning_speed, servo_deg=steer_servo)
            return

        # Default: normal or pillar-avoid
        servo = self.servo_center
        speed = self.base_speed

        # adaptive base speed by steering demand and front clearance
        front_clear = self.last_front_dist if np.isfinite(self.last_front_dist) else 1.0
        steer_mag = abs(servo - self.servo_center)
        steer_factor = max(0.55, 1.0 - 0.006 * steer_mag)
        dist_factor = min(1.0, 0.55 + 0.45 * (front_clear / 1.0))
        speed = int(self.base_speed * min(steer_factor, dist_factor))
        
        # Check if vision is ready (prevents relying on vision before it's initialized)
        if self.vision_enabled and not self.vision_ready:
            elapsed = time.time() - self.get_clock().now().nanoseconds / 1e9
            if elapsed > 2.0:  # Warn if vision not ready after 2 seconds
                self.get_logger().warning("⏳ Vision not ready yet - waiting for first frame...")
            # Don't use pillar avoidance until vision is ready
            self._set_and_send(speed=speed, servo_deg=servo)
            return

        # Persistent pillar avoidance steering if active
        if self.vision_enabled and self.vision_ready and self.pillar_avoidance_active:
            # EMERGENCY CHECK DURING AVOIDANCE: Still too close to obstacle?
            # Re-check distance to ensure we're not stuck or getting closer
            lidar_dist_check, lidar_width_check, lidar_is_obstacle_check = self._detect_lidar_obstacle()
            if (lidar_is_obstacle_check and 
                lidar_dist_check is not None and 
                lidar_dist_check < self.EMERGENCY_TOO_CLOSE_DISTANCE and
                lidar_width_check is not None and 
                lidar_width_check < self.WALL_WIDTH_THRESHOLD):
                
                # STILL TOO CLOSE during avoidance! Trigger emergency reverse
                self.get_logger().warning(
                    f"🚨 EMERGENCY DURING AVOIDANCE! Obstacle still too close: {lidar_dist_check:.3f}m < "
                    f"{self.EMERGENCY_TOO_CLOSE_DISTANCE:.2f}m → EMERGENCY REVERSE!"
                )
                self._start_emergency_reverse()
                return
            
            # PRIORITY: Use pillar avoidance servo as BASE direction
            servo = float(self.pillar_avoid_direction)
            speed = int(self.base_speed * 0.7)
            
            # micro correction toward target zone while locked (±2°)
            try:
                with self.frame_lock:
                    dets = [d for d in self.current_detections if self.tracked_pillar and d['color'] == self.tracked_pillar['color']]
                if dets:
                    w = self.latest_frame.shape[1] if self.latest_frame is not None else 320
                    left_end = int(w * self.ZONE_LEFT_FRAC)
                    right_start = int(w * self.ZONE_MIDDLE_FRAC)
                    target_cx = left_end // 2 if self.tracked_pillar and self.tracked_pillar.get('color') == 'red' else (right_start + (w - right_start) // 2)
                    x1, y1b, x2, y2b = dets[0]['bbox']
                    cx = (x1 + x2) // 2
                    err = (target_cx - cx) / max(1, w)
                    micro = max(-2.0, min(2.0, 40.0 * err))
                    servo = float(servo + micro)
            except:
                pass

            # OPTIMIZED: Throttle pillar avoidance status logging (was logging at 20Hz!)
            direction = "LEFT ⬅️" if servo < 90 else "RIGHT ➡️" if servo > 90 else "CENTER"
            self._log_throttled('pillar_active', f"🎯 PILLAR AVOIDANCE ACTIVE: {direction} servo={servo:.1f}°, speed={speed}", interval=0.5)

            # Safety: if no detections recently, allow timeout
            if time.time() - self.pillar_avoidance_start_time > 2.2:
                self.pillar_avoidance_active = False
                self.pillar_avoid_direction = self.servo_center
                self.tracked_pillar = None  # Clear tracked pillar
                self.pillar_avoidance_cooldown = time.time() + 1.0  # Shorter cooldown (was 2.0s)
                self.get_logger().info("⏱️ Pillar avoidance timeout - no recent detections, cooldown 1.0s")

        # Side-wall distance-based adjustments
        # During pillar avoidance: Apply SMALL adjustments on top of pillar steering to help with distance
        # During normal driving: Apply full wall corrections
        now_time = time.time()
        
        # Always get side distances for logging purposes
        left_dist, right_dist = self._get_side_distances()
        left_close = (left_dist is not None and left_dist < self.SIDE_NEAR_THRESHOLD)
        right_close = (right_dist is not None and right_dist < self.SIDE_NEAR_THRESHOLD)
        
        # Check if we're in initial pillar avoidance phase (first 1.0s where we ignore walls completely)
        in_initial_avoidance_phase = (
            self.pillar_avoidance_active and 
            now_time <= self.side_wall_disabled_until
        )
        
        if approaching_corner and (left_close or right_close):
            # DISABLED: Approaching corner - let corner detection handle it
            left_str = f"{left_dist:.2f}m" if left_dist else 'N/A'
            right_str = f"{right_dist:.2f}m" if right_dist else 'N/A'
            self.get_logger().info(
                f"🚦 APPROACHING CORNER - ignoring side wall corrections (L={left_str}, R={right_str})"
            )
        elif in_initial_avoidance_phase and (left_close or right_close):
            # DISABLED: Initial pillar avoidance phase - let pillar steering establish direction first
            left_str = f"{left_dist:.2f}m" if left_dist else 'N/A'
            right_str = f"{right_dist:.2f}m" if right_dist else 'N/A'
            self.get_logger().info(
                f"🎯 PILLAR AVOIDANCE INIT - ignoring walls to establish direction first (L={left_str}, R={right_str})"
            )
        elif (left_close or right_close):
            # APPLY WALL CORRECTIONS:
            # - During pillar avoidance: SMALL adjustments (±2-3°) on top of pillar steering
            # - During normal driving: FULL corrections (±5°)
            # suppress if a narrow obstacle is close directly ahead
            obs_dist, obs_w, obs_narrow = self._detect_lidar_obstacle()
            if obs_narrow and obs_dist is not None and obs_dist < 0.35:
                self._log_throttled('wall_suppressed', "🧯 Suppressing wall correction: close narrow obstacle ahead", interval=0.5)
            else:
                wall_correction, log_msg = self._compute_wall_correction(left_dist, right_dist)
                if self.pillar_avoidance_active:
                    if wall_correction > 2.0:
                        wall_correction = 2.0
                    elif wall_correction < -2.0:
                        wall_correction = -2.0
                    servo = float(servo) + wall_correction
                    # OPTIMIZED: Throttle wall correction logging
                    self._log_throttled('wall_assist', f"{log_msg} [ASSIST: +{wall_correction:+.1f}° to pillar {self.pillar_avoid_direction:.1f}° → final={servo:.1f}°]", interval=0.5)
                else:
                    servo = float(servo) + wall_correction
                    # OPTIMIZED: Throttle wall correction logging
                    self._log_throttled('wall_correct', f"{log_msg} → servo={servo:.1f}°", interval=0.5)

        # Clamp and send
        servo = max(self.servo_min, min(self.servo_max, servo))
        
        # Slew-rate limit servo to prevent twitch/wobble
        target_servo = float(max(self.servo_min, min(self.servo_max, servo)))
        try:
            prev_servo = float(self.last_servo_deg)
        except Exception:
            prev_servo = float(self.servo_center)
        servo = self._slew_servo(prev_servo, target_servo)
        
        # OPTIMIZED: Only log final command when something interesting is happening, and throttle it
        if self.pillar_avoidance_active or left_close or right_close:
            self._log_throttled('final_cmd', f"📤 FINAL COMMAND: servo={servo:.1f}°, speed={speed}", interval=0.5)
        self._set_and_send(speed=speed, servo_deg=servo)

    # ===================== Emergency obstacle reverse =====================
    def _start_emergency_reverse(self):
        """Start emergency reverse when too close to obstacle"""
        self.performing_emergency_reverse = True
        self.emergency_reverse_start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Determine target servo angle based on current detections
        try:
            with self.frame_lock:
                detections = self.current_detections.copy() if hasattr(self, 'current_detections') else []
            if detections:
                closest = min(detections, key=lambda d: d['distance'])
                # Set target servo to avoid in correct direction after reversing (AGGRESSIVE angles)
                if closest['color'] == 'green':
                    # Green → turn LEFT after reverse (AGGRESSIVE)
                    self.emergency_reverse_target_servo = 40
                else:  # red
                    # Red → turn RIGHT after reverse (AGGRESSIVE)
                    self.emergency_reverse_target_servo = 140
                self.get_logger().info(
                    f"🎯 Emergency target: {closest['color'].upper()} → "
                    f"servo will be {self.emergency_reverse_target_servo}° after reverse"
                )
            else:
                self.emergency_reverse_target_servo = 90
        except:
            self.emergency_reverse_target_servo = 90
        
        # Disable pillar avoidance during emergency reverse
        if self.pillar_avoidance_active:
            self.pillar_avoidance_active = False
            self.tracked_pillar = None  # Clear tracked pillar
            self.get_logger().info("🛑 Disabling pillar avoidance for emergency reverse")
        
        self.get_logger().warning(
            f"🚨 EMERGENCY REVERSE: backing up for {self.EMERGENCY_REVERSE_DURATION:.1f}s @ {self.EMERGENCY_REVERSE_SPEED}"
        )
    
    def _handle_emergency_reverse(self):
        """Handle emergency reverse maneuver"""
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - (self.emergency_reverse_start_time or now)
        
        if elapsed < self.EMERGENCY_REVERSE_DURATION:
            # Reversing straight back
            servo = self.EMERGENCY_REVERSE_SERVO
            speed = self.EMERGENCY_REVERSE_SPEED
            self._send_can(speed=speed, servo_deg=servo)
            self.get_logger().info(
                f"⏪ Emergency reverse: servo={servo}°, speed={speed}, t={elapsed:.2f}s"
            )
        else:
            # Reverse complete - resume with target servo angle
            self.performing_emergency_reverse = False
            self.get_logger().info(
                f"✅ Emergency reverse complete → resuming with servo={self.emergency_reverse_target_servo}°"
            )
            # Apply the target servo immediately
            with self._state_lock:
                self.last_servo_deg = float(self.emergency_reverse_target_servo)
                self.last_speed_cps = int(self.base_speed * 0.6)
            self._send_can(speed=self.last_speed_cps, servo_deg=self.last_servo_deg)

    # ===================== Backoff (reverse) =====================
    def _start_backoff(self):
        self.performing_backoff = True
        self.backoff_stage = 1
        self.backoff_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"↩️ Corner detected → Stage1: pre-turn RIGHT {self.PRETURN_SERVO}° for {self.PRETURN_DURATION:.1f}s (avoid inside wall), "
            f"Stage2: reverse {self.BACKOFF_DURATION:.1f}s (servo={self.BACKOFF_SERVO}°), "
            f"Stage3: straight forward {self.STRAIGHT_DURATION:.1f}s."
        )

    def _handle_backoff(self):
        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - (self.backoff_start_time or now)
        if self.backoff_stage == 1:
            servo = self.PRETURN_SERVO
            speed = self.PRETURN_SPEED
            self._send_can(speed=speed, servo_deg=servo)
            self.get_logger().info(f"↩️ Backoff Stage1: Servo={servo}°, Speed={speed}, t={elapsed:.2f}s")
            if elapsed >= self.PRETURN_DURATION:
                self.backoff_stage = 2
                self.backoff_start_time = now
        elif self.backoff_stage == 2:
            # Reverse with right-wall following
            right_dist = self._get_right_wall_distance()
            servo = self.BACKOFF_SERVO
            if right_dist is not None:
                error = right_dist - self.TARGET_RIGHT_DISTANCE
                servo += self.REVERSE_KP * error
                servo = max(35, min(145, servo))
                self.get_logger().info(
                    f"↩️ Backoff Stage2: Right={right_dist:.3f}m, Servo={servo:.1f}°, Speed={self.BACKOFF_SPEED}, t={elapsed:.2f}s"
                )
            else:
                self.get_logger().info(
                    f"↩️ Backoff Stage2 (no right wall): Servo={servo}°, Speed={self.BACKOFF_SPEED}, t={elapsed:.2f}s"
                )
            self._send_can(speed=self.BACKOFF_SPEED, servo_deg=servo)
            if elapsed >= self.BACKOFF_DURATION:
                self.backoff_stage = 3
                self.backoff_start_time = now
        elif self.backoff_stage == 3:
            # NEW STAGE: Go straight forward after reverse
            servo = self.STRAIGHT_SERVO
            speed = self.STRAIGHT_SPEED
            self._send_can(speed=speed, servo_deg=servo)
            self.get_logger().info(f"➡️ Backoff Stage3 (STRAIGHT FORWARD): Servo={servo}°, Speed={speed}, t={elapsed:.2f}s")
            if elapsed >= self.STRAIGHT_DURATION:
                self.backoff_stage = 4
                self.backoff_start_time = now
        elif self.backoff_stage == 4:
            servo = self.CENTER_SERVO
            speed = self.base_speed
            self._send_can(speed=speed, servo_deg=servo)
            self.get_logger().info(f"↩️ Backoff Stage4: Servo={servo}°, Speed={speed}, t={elapsed:.2f}s")
            if elapsed >= self.CENTER_DURATION:
                self.performing_backoff = False
                self.backoff_stage = 0
                with self._state_lock:
                    self.last_servo_deg = self.servo_center
                    self.last_speed_cps = self.base_speed
                self.get_logger().info("✅ Backoff complete — resuming normal operation.")

    # ===================== CAN helpers =====================
    def _send_can(self, speed: int, servo_deg: float):
        servo_raw = int(round(servo_deg * 100.0))
        data = struct.pack(">ii", int(speed), int(servo_raw))
        try:
            msg = can.Message(arbitration_id=0x101, data=data, is_extended_id=False)
            if self.bus is not None:
                with self._can_send_lock:
                    self.bus.send(msg)
        except can.CanError as e:
            self.get_logger().warning(f"CAN send error: {e}")

    # ===================== Side distance helpers =====================
    def _get_side_distances(self):
        """
        OPTIMIZED: Cache side distances per scan to avoid recalculating multiple times per control loop.
        This function is called 3-5 times per control loop (20Hz) with the same scan data!
        """
        msg = self.last_scan_msg
        if msg is None:
            return None, None
        
        # Use scan timestamp as cache key (each scan has unique timestamp)
        scan_id = id(msg)  # Use object id as unique identifier
        
        # Return cached value if same scan
        if scan_id == self._cached_side_distances_scan_id:
            return self._cached_side_distances
        
        # Calculate and cache
        degs = self._build_signed_deg_array(msg)
        rngs = np.array(msg.ranges, dtype=np.float32)
        m_left = self._sector_mask_signed(degs, *self.SIDE_LEFT_DEG)
        m_right = self._sector_mask_signed(degs, *self.SIDE_RIGHT_DEG)
        d_left = self._robust_sector_distance(rngs, m_left)
        d_right = self._robust_sector_distance(rngs, m_right)
        if not np.isfinite(d_left):
            d_left = None
        if not np.isfinite(d_right):
            d_right = None
        
        # Light exponential smoothing to reduce wobble
        prev_left, prev_right = self._side_smooth_prev
        alpha = 0.35
        if d_left is not None:
            d_left = d_left if prev_left is None else (prev_left + alpha * (d_left - prev_left))
        if d_right is not None:
            d_right = d_right if prev_right is None else (prev_right + alpha * (d_right - prev_right))
        self._side_smooth_prev = (d_left, d_right)
        # Update cache
        self._cached_side_distances = (d_left, d_right)
        self._cached_side_distances_scan_id = scan_id        
        return d_left, d_right
    # ===================== High-level control helpers =====================
    def stop_robot(self):
        try:
            servo_raw = int(round(self.servo_center * 100.0))
            data = struct.pack(">ii", 0, servo_raw)
            msg = can.Message(arbitration_id=0x101, data=data, is_extended_id=False)
            if self.bus is not None:
                with self._can_send_lock:
                    self.bus.send(msg)
            with self._state_lock:
                self.last_speed_cps = 0
                self.last_servo_deg = self.servo_center
        except Exception as e:
            self.get_logger().warning(f"Stop error: {e}")

    def initialize_robot(self):
        # Reset runtime flags/state; do not send motion until control loop
        self.performing_backoff = False
        self.backoff_stage = 0
        self.pillar_avoidance_active = False
        self.pillar_avoid_direction = self.servo_center
        self.tracked_pillar = None  # Clear tracked pillar
        self.pillar_avoidance_cooldown = 0.0
        self.side_wall_disabled_until = 0.0
        with self._state_lock:
            self.last_servo_deg = self.servo_center
            self.last_speed_cps = self.base_speed

    # ===================== Cleanup =====================
    def cleanup(self):
        try:
            self._vision_stop.set()
        except Exception:
            pass
        try:
            if self._vision_thread is not None:
                self._vision_thread.join(timeout=1.0)
        except Exception:
            pass
        # OPTIMIZED: Stop frame saver thread and wait for queue to flush
        try:
            self._frame_saver_stop.set()
            # Put poison pill to wake up thread if it's waiting
            try:
                self._frame_save_queue.put_nowait(None)
            except:
                pass
            if self._frame_saver_thread is not None:
                self._frame_saver_thread.join(timeout=2.0)
        except Exception:
            pass
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
                self.get_logger().info("RealSense pipeline stopped")
        except Exception as e:
            self.get_logger().warning(f"RealSense cleanup error: {e}")
        try:
            if self.bus is not None:
                self.bus.shutdown()
        except Exception:
            pass
def main(args=None):
    rclpy.init(args=args)
    node = CombinedCornerReverseAndPillarAvoid()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Keyboard interrupt - stopping")
        try:
            node.stop_robot()
        except Exception:
            pass
        try:
            node.initialize_robot()
        except Exception:
            pass
    finally:
        try:
            node.cleanup()
        except Exception as e:
            try:
                node.get_logger().warning(f"Cleanup error: {e}")
            except Exception:
                pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
if __name__ == '__main__':
    main()