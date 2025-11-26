#!/usr/bin/env python3

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
from sensor_msgs.msg import LaserScan
from datetime import datetime

os.environ['YOLO_VERBOSE'] = 'False'
os.environ['ULTRALYTICS_VERBOSE'] = 'False'

from ultralytics import YOLO


class CombinedCornerReverseAndPillarAvoid(Node):

    MODEL_PATH = "/home/aupp/ros2_ws/src/lidar_controller/lidar_controller/best.pt"
    CONF = 0.8
    IOU = 0.50

    ROI_X = (0.05, 0.95)    # Horizontal ROI: left edge (0%) to right edge (100%)
    ROI_Y = (0.20, 0.80)  # Vertical ROI: top (30%) to bottom (70%)
    ZONE_LEFT_FRAC = 0.30
    ZONE_MIDDLE_FRAC = 0.70
    
    CORNER_DETECTION_COOLDOWN = 10.0  # Cooldown for corner detection

    FRONT_DEG = (-20.0, +20.0)
    RANGE_MIN = 0.12
    RANGE_MAX = 4.0
    MIN_HITS = 10
    K_FRACT = 0.2

    OBSTACLE_DETECT_DISTANCE = 1.0
    WALL_WIDTH_THRESHOLD = 0.08
    
    FRONT_CORNER_WINDOW = (0.30, 0.50)
    LEFT_NEAR_THRESHOLD = 0.70

    SIDE_LEFT_DEG = (35.0, 75.0)
    SIDE_RIGHT_DEG = (-75.0, -30.0)
    SIDE_NEAR_THRESHOLD = 0.55
    SIDE_STEER_DELTA = 10.0

    # Progressive wall avoidance system (3 levels: gentle → warning → PANIC)
    WARNING_WALL_DISTANCE = 0.45        # moderate correction when wall is getting close
    WARNING_SERVO_LEFT = 60.0           # moderate left turn when right wall at warning distance
    WARNING_SERVO_RIGHT = 120.0         # moderate right turn when left wall at warning distance
    
    PANIC_WALL_DISTANCE = 0.20          # HARD override when wall is dangerously close
    PANIC_SERVO_HARD_LEFT = 35.0        # AGGRESSIVE left when right wall is too close (was 45.0)
    PANIC_SERVO_HARD_RIGHT = 145.0      # AGGRESSIVE right when left wall is too close (was 135.0)

    SERVO_MAX_SLEW_DEG_PER_SEC = 120.0
    
    GREEN_AGGRESSIVE = 40
    GREEN_NORMAL = 60
    GREEN_GENTLE = 80
    
    RED_AGGRESSIVE = 150
    RED_NORMAL = 135
    RED_GENTLE = 120

    PRETURN_DURATION = 1.3
    PRETURN_SERVO = 145
    PRETURN_SPEED = 450

    BACKOFF_DURATION = 2.4
    BACKOFF_SERVO = 30
    BACKOFF_SPEED = -1500
    
    CENTER_DURATION = 0.5
    CENTER_SERVO = 90

    FRONT_SCAN_ANGLE_RANGE_DEG = 20
    FRONT_MAX_RANGE = 1.3

    TARGET_LEFT_DISTANCE = 0.5
    REVERSE_KP = 50.0
    
    CAR_WIDTH = 0.25
    MIN_SIDE_CLEARANCE = 0.10

    def __init__(self):
        super().__init__('combined_corner_reverse_and_pillar_avoid')

        self.base_speed = 400
        self.min_speed = 300
        self.servo_center = 90.0
        self.servo_min = 30.0
        self.servo_max = 150.0
        self.send_rate_hz = 20.0

        self.last_servo_deg = self.servo_center
        self.last_speed_cps = self.min_speed
        self._state_lock = threading.Lock()

        self.last_scan_time = None
        self.last_scan_msg = None
        self.last_front_dist = float('nan')

        self.performing_backoff = False
        self.backoff_stage = 0
        self.backoff_start_time = None

        self.pillar_avoidance_active = False
        self.pillar_avoidance_start_time = 0.0
        self.pillar_avoid_direction = self.servo_center
        self.tracked_pillar = None
        self.pillar_avoidance_cooldown = 0.0
        self.current_detections = []
        self.side_wall_disabled_until = 0.0
        self.obstacle_queue = []
        
        self.corner_detection_cooldown = 0.0  # Corner detection cooldown
        
        self.vision_ready = False
        self.vision_ready_time = 0.0
        
        self._vision_heartbeat_last_log = 0.0
        self._vision_first_frame_logged = False

        self.cap = None
        self.frame_lock = threading.Lock()
        self.latest_frame = None
        self.latest_depth_frame = None
        self._vision_stop = threading.Event()
        self._vision_thread = None
        
        self.get_logger().info(f"Loading YOLO model: {self.MODEL_PATH}")
        self.model = YOLO(self.MODEL_PATH)
        try:
            self.model.overrides['task'] = 'detect'
        except Exception:
            pass
        
        self.yolo_device = 'auto'
        self.yolo_half = False
        
        self.get_logger().info("🔥 Warming up YOLO model (first inference)...")
        try:
            import numpy as np
            dummy_frame = np.zeros((240, 320, 3), dtype=np.uint8)
            _ = self.model(dummy_frame, verbose=False)
            self.get_logger().info("✅ YOLO warmup complete - model ready for fast inference!")
        except Exception as e:
            self.get_logger().warning(f"YOLO warmup failed (will warmup on first real frame): {e}")
        
        self.pipeline = None
        self.depth_scale = None
        self.vision_enabled = False
        
        candidate_res = [(640, 480, 30)]
        ctx = rs.context()
        
        if len(ctx.devices) == 0:
            self.get_logger().warning("No RealSense device connected; pillar avoidance disabled")
        else:
            self.pipeline = rs.pipeline()
            profile = None
            
            for (w, h, fps) in candidate_res:
                try:
                    config = rs.config()
                    config.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
                    config.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
                    profile = self.pipeline.start(config)
                    self.get_logger().info(f"RealSense started: {w}x{h}@{fps}fps (color+depth)")
                    self.vision_enabled = True
                    break
                except RuntimeError as e:
                    try:
                        self.pipeline.stop()
                    except Exception:
                        pass
                    self.get_logger().warning(f"RealSense {w}x{h}@{fps}fps failed: {e}")
            
            if profile is None:
                self.get_logger().warning("Failed to start RealSense; pillar avoidance disabled")
                self.pipeline = None
            else:
                try:
                    depth_sensor = profile.get_device().first_depth_sensor()
                    self.depth_scale = depth_sensor.get_depth_scale()
                    self.get_logger().info(f"Depth scale: {self.depth_scale:.6f} m/unit")
                except Exception as e:
                    self.get_logger().warning(f"Could not get depth scale: {e}")
                    self.depth_scale = 0.001
                
                self._vision_thread = threading.Thread(target=self._vision_loop, daemon=True)
                self._vision_thread.start()

        self.bus = None
        self._can_send_lock = threading.Lock()
        self._can_send_fail_count = 0
        try:
            self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=250000)
            self.get_logger().info("CAN bus initialized on can0 @250k")
        except Exception as e:
            self.bus = None
            self.get_logger().warning(f"CAN bus init failed: {e}")

        self.scan_sub = self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.cmd_timer = self.create_timer(1.0 / self.send_rate_hz, self._control_loop)

        self.get_logger().info("✅ Combined controller ready: normal → pillar avoid → corner reverse")

    def _get_median_depth(self, depth_frame, x1, y1, x2, y2):
        if not depth_frame or self.depth_scale is None:
            return None
        depth_image = np.asanyarray(depth_frame.get_data())
        h, w = depth_image.shape
        x1, x2 = max(0, x1), min(w, x2)
        y1, y2 = max(0, y1), min(h, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        patch = depth_image[y1:y2, x1:x2]
        valid = patch[patch > 0]
        if len(valid) == 0:
            return None
        median_depth = np.median(valid) * self.depth_scale
        return median_depth

    def _vision_loop(self):
        target_hz = 30.0
        dt = 1.0 / target_hz
        frame_count = 0
        session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        capture_dir = f"/home/aupp/ros2_ws/src/lidar_controller/lidar_controller/capture/session_{session_timestamp}"
        os.makedirs(capture_dir, exist_ok=True)
        self.get_logger().info(f"📁 Captures will be saved to: {capture_dir}")
        
        def process_frame():
            nonlocal frame_count
            if self._vision_stop.is_set() or not rclpy.ok():
                return
            
            frame = None
            depth_frame = None
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=100)
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if color_frame:
                    frame = np.asanyarray(color_frame.get_data())
            except Exception as e:
                self.get_logger().warning(f"RealSense read error: {e}")
                frame = None
                depth_frame = None
            
            if frame is None:
                threading.Timer(0.02, process_frame).start()
                return

            frame_count += 1
            
            if not self._vision_first_frame_logged:
                self.get_logger().info("✅ Vision first frame captured - camera working!")
                self._vision_first_frame_logged = True
                self.vision_ready = True
                self.vision_ready_time = time.time()
                self.get_logger().info("🟢 Vision system READY - obstacle detection active")
            
            now = time.time()
            if now - self._vision_heartbeat_last_log >= 2.0:
                self.get_logger().info(f"💓 Vision heartbeat: frame #{frame_count}, camera active")
                self._vision_heartbeat_last_log = now

            try:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.latest_depth_frame = depth_frame
            except Exception:
                pass

            try:
                self._process_frame_for_pillars(frame, depth_frame)
            except Exception as e:
                self.get_logger().error(f"❌ Vision processing error: {e}")

            if frame_count % 10 == 0:
                try:
                    annotated_frame = frame.copy()
                    
                    try:
                        with self.frame_lock:
                            detections = self.current_detections.copy() if hasattr(self, 'current_detections') else []
                    except:
                        detections = []
                    
                    h, w = annotated_frame.shape[:2]
                    # Draw ROI rectangle with both X and Y boundaries
                    roi_x1, roi_x2 = int(w * self.ROI_X[0]), int(w * self.ROI_X[1])
                    roi_y1, roi_y2 = int(h * self.ROI_Y[0]), int(h * self.ROI_Y[1])
                    cv2.rectangle(annotated_frame, (roi_x1, roi_y1), (roi_x2, roi_y2), (255, 255, 0), 2)
                    cv2.putText(annotated_frame, "ROI", (roi_x1 + 5, roi_y1 - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
                    
                    # Zone lines are relative to ROI width
                    roi_width = roi_x2 - roi_x1
                    left_zone_x = roi_x1 + int(roi_width * self.ZONE_LEFT_FRAC)
                    right_zone_x = roi_x1 + int(roi_width * self.ZONE_MIDDLE_FRAC)
                    
                    cv2.line(annotated_frame, (left_zone_x, roi_y1), (left_zone_x, roi_y2), (0, 255, 255), 2)
                    cv2.putText(annotated_frame, "LEFT|MID", (left_zone_x - 60, roi_y1 + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    
                    cv2.line(annotated_frame, (right_zone_x, roi_y1), (right_zone_x, roi_y2), (0, 255, 255), 2)
                    cv2.putText(annotated_frame, "MID|RIGHT", (right_zone_x - 60, roi_y1 + 20), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                    
                    for idx, det in enumerate(detections):
                        if 'bbox' in det:
                            x1, y1, x2, y2 = det['bbox']
                            
                            # Convert ROI-relative coordinates to full frame coordinates
                            x1_full = x1 + roi_x1
                            x2_full = x2 + roi_x1
                            y1_full = y1 + roi_y1
                            y2_full = y2 + roi_y1
                            
                            color_bgr = (0, 0, 255) if det['color'] == 'red' else (0, 255, 0)
                            
                            cv2.rectangle(annotated_frame, (x1_full, y1_full), (x2_full, y2_full), color_bgr, 3)
                            
                            zone_text = det['zone'].upper()
                            if 'distance' in det and np.isfinite(det['distance']) and det['distance'] < 10.0:
                                dist_text = f"LiDAR:{det['distance']:.2f}m"
                                label = f"{det['color'].upper()} {zone_text} {dist_text}"
                            else:
                                label = f"{det['color'].upper()} {zone_text}"
                            
                            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                            cv2.rectangle(annotated_frame, (x1_full, y1_full - th - 10), (x1_full + tw + 10, y1_full), color_bgr, -1)
                            cv2.putText(annotated_frame, label, (x1_full + 5, y1_full - 5), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    
                    info_text = f"Frame: {frame_count} | Detections: {len(detections)}"
                    cv2.putText(annotated_frame, info_text, (10, frame.shape[0] - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    status_text = f"Pillar Avoid: {self.pillar_avoidance_active}"
                    cv2.putText(annotated_frame, status_text, (10, frame.shape[0] - 30), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    if self.tracked_pillar:
                        tracked_text = f"Tracked: {self.tracked_pillar['color']} in {self.tracked_pillar['zone']}"
                        cv2.putText(annotated_frame, tracked_text, (10, frame.shape[0] - 50), 
                                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    
                    capture_path = f"{capture_dir}/frame_{frame_count:06d}.jpg"
                    cv2.imwrite(capture_path, annotated_frame)
                    self.get_logger().info(f"📸 Saved annotated frame: {capture_path} ({len(detections)} detections)")
                except Exception as e:
                    self.get_logger().warning(f"Capture save error: {e}")

            threading.Timer(dt, process_frame).start()
        
        process_frame()

    def _process_frame_for_pillars(self, frame, depth_frame=None):
        h, w = frame.shape[:2]
        # Apply both horizontal (X) and vertical (Y) ROI
        x1, x2 = int(w * self.ROI_X[0]), int(w * self.ROI_X[1])
        y1, y2 = int(h * self.ROI_Y[0]), int(h * self.ROI_Y[1])
        roi_frame = frame[y1:y2, x1:x2]  # Crop to ROI rectangle
        
        self.get_logger().info(f"🎬 Processing frame: {w}x{h} → ROI: {roi_frame.shape[1]}x{roi_frame.shape[0]} (x:{x1}-{x2}, y:{y1}-{y2})")

        try:
            results = self.model.predict(roi_frame, conf=self.CONF, iou=self.IOU, verbose=False)
        except Exception as e:
            self.get_logger().error(f"❌ YOLO prediction failed: {e}")
            return
            
        if len(results) == 0:
            self.get_logger().info("⚠️ No YOLO results returned")
            with self.frame_lock:
                self.current_detections = []
            return
            
        r = results[0]
        boxes = r.boxes

        detected = []
        
        class_names = getattr(r, 'names', None) or getattr(self.model, 'names', None) or {}
        self.get_logger().info(f"📋 YOLO class names: {class_names}")
        
        if boxes is not None and len(boxes) > 0:
            xyxys = boxes.xyxy.cpu().numpy() if boxes.xyxy is not None else []
            confs = boxes.conf.cpu().numpy() if boxes.conf is not None else []
            class_ids = boxes.cls.cpu().numpy().astype(int) if boxes.cls is not None else []
            
            self.get_logger().info(f"🔍 YOLO found {len(boxes)} object(s), class_ids: {class_ids}")
            
            img_w = roi_frame.shape[1]
            left_end = int(img_w * self.ZONE_LEFT_FRAC)
            right_start = int(img_w * self.ZONE_MIDDLE_FRAC)

            for i, (bb, conf, class_id) in enumerate(zip(xyxys, confs, class_ids)):
                x1, y1b, x2, y2b = map(int, bb)
                
                raw_color = class_names.get(class_id, f"class_{class_id}") if isinstance(class_names, dict) else str(class_id)
                self.get_logger().info(f"🔍 YOLO class_id={class_id}, raw_name='{raw_color}', type={type(raw_color)}")
                
                color_str = str(raw_color).lower().strip()
                
                if 'green' in color_str:
                    color = 'green'
                    self.get_logger().info(f"✅ Matched GREEN from '{raw_color}'")
                elif 'red' in color_str:
                    color = 'red'
                    self.get_logger().info(f"✅ Matched RED from '{raw_color}'")
                else:
                    self.get_logger().warning(f"⚠️ YOLO detected unknown class: '{raw_color}' (class_id={class_id}) - no 'red' or 'green' found - SKIPPING")
                    continue

                center_x = (x1 + x2) // 2
                if center_x <= left_end:
                    zone = 'left'
                elif center_x >= right_start:
                    zone = 'right'
                else:
                    zone = 'middle'

                depth_dist = None
                if depth_frame is not None:
                    # Adjust depth coordinates to account for X offset
                    depth_dist = self._get_median_depth(depth_frame, x1 + x1, y1b + y1, x2 + x1, y2b + y1)
                
                detected.append({
                    'color': color,
                    'zone': zone,
                    'conf': float(conf),
                    'bbox': (x1, y1b, x2, y2b),  # Coordinates relative to ROI
                    'center_x': center_x,
                    'depth_distance': depth_dist
                })
                
                depth_str = f"{depth_dist:.2f}m" if depth_dist is not None else "N/A"
                self.get_logger().info(f"👁️ Detected {color} pillar in {zone} zone (conf: {conf:.2f}, depth: {depth_str})")
        else:
            self.get_logger().info("⚠️ No boxes detected by YOLO (boxes is None or empty)")

        lidar_dist, lidar_width, lidar_is_obstacle = self._detect_lidar_obstacle()
        
        if lidar_dist is not None and np.isfinite(lidar_dist) and lidar_width is not None:
            self.get_logger().info(
                f"📡 LiDAR width check: {lidar_dist:.2f}m, width={lidar_width:.3f}m → "
                f"{'PILLAR' if lidar_is_obstacle else 'WALL'}"
            )

        if detected:
            img_w = roi_frame.shape[1]
            center_x = img_w // 2
            for det in detected:
                det['center_offset'] = abs(det['center_x'] - center_x)
            
            central_detection = min(detected, key=lambda x: x['center_offset'])
            
            for det in detected:
                if det is central_detection:
                    if det.get('depth_distance') is None or not np.isfinite(det.get('depth_distance', float('nan'))):
                        det['distance'] = float(lidar_dist) if (lidar_dist is not None and np.isfinite(lidar_dist)) else float('inf')
                    else:
                        det['distance'] = det['depth_distance']
                else:
                    det['distance'] = det.get('depth_distance') if (det.get('depth_distance') is not None and np.isfinite(det.get('depth_distance', float('nan')))) else float('inf')
                
                dist_source = "RS" if det.get('depth_distance') is not None else "LiDAR" if det.get('distance', float('inf')) < float('inf') else "N/A"
                depth_dist_val = det.get('depth_distance')
                distance_val = det.get('distance', float('inf'))
                
                depth_dist_str = f"{depth_dist_val:.2f}m" if depth_dist_val is not None and np.isfinite(depth_dist_val) else "N/A"
                distance_str = f"{distance_val:.2f}m" if distance_val is not None and np.isfinite(distance_val) and distance_val < float('inf') else "N/A"
                
                self.get_logger().info(
                    f"📏 {det['color']} in {det['zone']}: depth_distance={depth_dist_str}, "
                    f"distance={distance_str} (source: {dist_source})"
                )
            
            self.get_logger().info(f"🔍 Processing {len(detected)} detected obstacles for queueing...")
            
            for detection in detected:
                obstacle_already_known = False
                
                if self.tracked_pillar:
                    if (self.tracked_pillar['color'] == detection['color'] and 
                        self.tracked_pillar['zone'] == detection['zone']):
                        obstacle_already_known = True
                        self.get_logger().info(
                            f"📋 {detection['color']} in {detection['zone']} already TRACKED - skipping"
                        )
                        continue
                
                if not obstacle_already_known:
                    for idx, queued in enumerate(self.obstacle_queue):
                        if (queued['detection']['color'] == detection['color'] and 
                            queued['detection']['zone'] == detection['zone']):
                            old_dist = queued['detection'].get('distance', float('inf'))
                            new_dist = detection.get('distance', float('inf'))
                            if new_dist < old_dist:
                                self.obstacle_queue[idx]['detection'] = detection
                                self.obstacle_queue[idx]['lidar_dist'] = lidar_dist if detection is central_detection else None
                                self.obstacle_queue[idx]['lidar_width'] = lidar_width if detection is central_detection else None
                                
                                old_dist_str = f"{old_dist:.2f}m" if old_dist is not None and np.isfinite(old_dist) and old_dist < float('inf') else "N/A"
                                new_dist_str = f"{new_dist:.2f}m" if new_dist is not None and np.isfinite(new_dist) and new_dist < float('inf') else "N/A"
                                
                                self.get_logger().info(
                                    f"🔄 Updated queued {detection['color']} in {detection['zone']}: "
                                    f"{old_dist_str} → {new_dist_str} (closer!)"
                                )
                            else:
                                self.get_logger().info(
                                    f"📋 {detection['color']} in {detection['zone']} already queued"
                                )
                            obstacle_already_known = True
                            break
                
                if not obstacle_already_known:
                    rs_depth = detection.get('depth_distance')
                    check_dist = detection.get('distance', float('inf'))
                    
                    is_valid_obstacle = (detection is central_detection and lidar_is_obstacle) or (rs_depth is not None and np.isfinite(rs_depth))
                    
                    if is_valid_obstacle and check_dist <= self.OBSTACLE_DETECT_DISTANCE:
                        source = "RealSense" if (rs_depth is not None and np.isfinite(rs_depth)) else "LiDAR"
                        obstacle_type = "PILLAR(LiDAR)" if (detection is central_detection and lidar_is_obstacle) else "PILLAR(Vision)"
                        
                        can_pass, left_gap, right_gap, clearance_reason = self._check_obstacle_clearance(lidar_width) if detection is central_detection else (True, None, None, "Non-central (RealSense only)")
                        
                        if can_pass or detection is not central_detection:
                            if detection is central_detection:
                                self.get_logger().info(f"✅ CLEARANCE: {clearance_reason}")
                        else:
                            self.get_logger().warning(f"⚠️ TIGHT: {clearance_reason}")
                        
                        self.obstacle_queue.append({
                            'detection': detection,
                            'lidar_dist': lidar_dist if detection is central_detection else None,
                            'lidar_width': lidar_width if detection is central_detection else None,
                            'can_pass': can_pass,
                            'left_gap': left_gap,
                            'right_gap': right_gap,
                            'timestamp': time.time()
                        })
                        
                        check_dist_str = f"{check_dist:.2f}m" if check_dist is not None and np.isfinite(check_dist) and check_dist < float('inf') else "N/A"
                        
                        self.get_logger().info(
                            f"📋 Queued {obstacle_type}: {detection['color']} in {detection['zone']} zone "
                            f"({source} dist={check_dist_str}). Total in queue: {len(self.obstacle_queue)}"
                        )
        
        if not self.pillar_avoidance_active and not self.tracked_pillar and self.obstacle_queue:
            current_time = time.time()
            if current_time >= self.pillar_avoidance_cooldown:
                self.obstacle_queue.sort(key=lambda x: x['detection'].get('distance', float('inf')))
                next_obstacle = self.obstacle_queue.pop(0)
                self._activate_pillar_avoidance(next_obstacle['detection'], next_obstacle['lidar_dist'], next_obstacle['lidar_width'])
                obstacle_dist = next_obstacle['detection'].get('distance', float('inf'))
                obstacle_dist_str = f"{obstacle_dist:.2f}m" if obstacle_dist is not None and np.isfinite(obstacle_dist) and obstacle_dist < float('inf') else "N/A"
                self.get_logger().info(
                    f"🎯 Activated CLOSEST obstacle: {next_obstacle['detection']['color']} at "
                    f"{obstacle_dist_str} in {next_obstacle['detection']['zone']} zone. "
                    f"Remaining in queue: {len(self.obstacle_queue)}"
                )
            else:
                remaining_cooldown = self.pillar_avoidance_cooldown - current_time
                self.get_logger().info(
                    f"⏳ Cooldown active: waiting {remaining_cooldown:.2f}s before activating next obstacle "
                    f"(queue: {len(self.obstacle_queue)})"
                )
        
        elif self.tracked_pillar and detected:
            tracked_color = self.tracked_pillar['color']
            tracked_detections = [d for d in detected if d['color'] == tracked_color]
            
            if tracked_detections:
                target_zone_reached = False
                for det in tracked_detections:
                    if tracked_color == 'green' and det['zone'] == 'right':
                        target_zone_reached = True
                        self.get_logger().info(f"✅ GREEN pillar reached RIGHT zone - mission accomplished!")
                        break
                    elif tracked_color == 'red' and det['zone'] == 'left':
                        target_zone_reached = True
                        self.get_logger().info(f"✅ RED pillar reached LEFT zone - mission accomplished!")
                        break
                
                if target_zone_reached:
                    # Set specific servo angles based on color when target zone is reached
                    if tracked_color == 'green':
                        steer_back_angle = self.servo_center + 15.0 # Green: slight right (center + 15°)
                        self.get_logger().info(f"🟢 Green pillar passed - steering to {steer_back_angle}° (slight right)")
                    else:  # red
                        steer_back_angle = self.servo_center - 15.0   # Red: slight left (center - 10°)
                        self.get_logger().info(f"🔴 Red pillar passed - steering to {steer_back_angle}° (slight left)")
                    
                    self.get_logger().info(f"👋 Releasing {tracked_color} pillar - target zone reached, ready for next obstacle")
                    self.tracked_pillar = None
                    self.pillar_avoidance_active = False
                    self.pillar_avoid_direction = steer_back_angle
                    self.pillar_avoidance_cooldown = time.time() + 0.2
                    self.get_logger().info("✅ Pillar avoidance deactivated - mission complete, returning to center")
                else:
                    self.tracked_pillar['timestamp'] = time.time()
                    
                    closest_det = min(tracked_detections, key=lambda d: d.get('depth_distance', d.get('distance', float('inf'))))
                    new_servo = self._pillar_avoidance_servo([closest_det])
                    if new_servo is not None:
                        self.pillar_avoid_direction = float(new_servo)
                        self.get_logger().info(
                            f"🔄 Tracking {tracked_color} pillar: zone={closest_det['zone']} → "
                            f"updated servo={self.pillar_avoid_direction:.0f}°"
                        )
                    else:
                        self.get_logger().info(f"🔒 Still tracking {tracked_color} pillar - {len(tracked_detections)} detected")
            else:
                self.get_logger().info(f"👋 Lost sight of {tracked_color} pillar - releasing tracking")
                self.tracked_pillar = None
        
        elif self.tracked_pillar and not detected:
            tracked_color = self.tracked_pillar.get('color', 'UNKNOWN')
            self.get_logger().info(f"👋 No pillars detected - releasing {tracked_color} tracking")
            self.tracked_pillar = None

        with self.frame_lock:
            self.current_detections = detected

    def _activate_pillar_avoidance(self, detection, lidar_dist, lidar_width):
        color = detection['color']
        zone = detection['zone']
        depth_dist = detection.get('depth_distance')
        actual_dist = detection.get('distance', float('inf'))
        
        self.pillar_avoidance_active = True
        self.pillar_avoidance_start_time = time.time()
        self.side_wall_disabled_until = time.time() + 1.0
        self.tracked_pillar = {
            'color': color, 
            'zone': zone, 
            'timestamp': time.time(),
            'activation_distance': lidar_dist if lidar_dist is not None else actual_dist,
            'depth_distance': depth_dist,
            'distance': actual_dist
        }
        
        servo = self._pillar_avoidance_servo([detection])
        if servo is not None:
            self.pillar_avoid_direction = float(servo)
        
        if color == 'green':
            direction_msg = "LEFT ⬅️" if servo < 90 else "CENTER" if servo == 90 else "RIGHT ➡️"
            target_zone = "RIGHT"
            strategy = f"Avoid {direction_msg} to push pillar from {zone.upper()} → {target_zone}"
        else:
            direction_msg = "RIGHT ➡️" if servo > 90 else "CENTER" if servo == 90 else "LEFT ⬅️"
            target_zone = "LEFT"
            strategy = f"Avoid {direction_msg} to push pillar from {zone.upper()} → {target_zone}"
        
        dist_source = "LiDAR" if lidar_dist is not None else "RealSense"
        dist_value = lidar_dist if lidar_dist is not None else actual_dist
        dist_str = f"{dist_value:.2f}m" if dist_value is not None and dist_value < float('inf') else "N/A"
        
        self.get_logger().info(
            f"🎯 OBSTACLE LOCK: {color.upper()} pillar at {dist_str} ({dist_source}) in {zone.upper()} zone "
            f"→ {strategy} → servo={self.pillar_avoid_direction:.0f}°"
        )

    def _pillar_avoidance_servo(self, pillars):
        """
        Adaptive pillar avoidance with aggression based on position urgency:
        - GENTLE: Pillar already in correct zone (just maintain)
        - MODERATE: Pillar in middle (needs to move)
        - AGGRESSIVE: Pillar in WRONG zone (urgent!)
        """
        if not pillars:
            return None
        
        p = pillars[0]
        color, zone = p['color'], p['zone']
        pillar_dist = p.get('distance', float('inf'))
        
        left_dist, right_dist = self._get_side_distances()
        
        if color == 'red':
            if right_dist is not None and right_dist < 0.35:
                if zone in ['left', 'middle']:
                    base_servo = 100
                    return base_servo
        if color == 'green':
            if zone == 'right':
                base_servo = self.GREEN_GENTLE
                aggression = "GENTLE"
                reason = "already on correct side (right)"
            elif zone == 'middle':
                base_servo = self.GREEN_NORMAL
                aggression = "MODERATE"
                reason = "needs to move right"
            else:
                base_servo = self.GREEN_AGGRESSIVE
                aggression = "AGGRESSIVE"
                reason = "WRONG SIDE (left) - urgent right turn!"
                
        else: #red
            if zone == 'left':
                base_servo = self.RED_GENTLE
                aggression = "GENTLE"
                reason = "already on correct side (left)"
            elif zone == 'middle':
                base_servo = self.RED_NORMAL
                aggression = "MODERATE"  
                reason = "needs to move left"
            else:
                base_servo = self.RED_AGGRESSIVE
                aggression = "AGGRESSIVE"
                reason = "WRONG SIDE (right) - urgent left turn!"

        if color == 'red' and right_dist is not None and right_dist < 0.4:
            base_servo = max(base_servo, 100)
        if color == 'green' and left_dist is not None and left_dist < 0.4:
            base_servo = max(base_servo, 80)
        
        if pillar_dist is not None and pillar_dist < float('inf'):
            if pillar_dist < 0.5:
                if aggression == "AGGRESSIVE":
                    base_servo = base_servo + (5 if color == 'red' else -5)
                    reason += f" + VERY CLOSE ({pillar_dist:.2f}m) → EXTRA aggressive!"
            elif pillar_dist > 0.8:
                if aggression == "GENTLE" and zone in ['right', 'left']:
                    base_servo = 90
                    reason += f" + FAR ({pillar_dist:.2f}m) → CENTER (coast)"
        
        if aggression == "GENTLE":
            if color == 'green' and left_dist and left_dist < 0.4:
                base_servo = 85
                reason += " + left wall close → softer"
            elif color == 'red' and right_dist and right_dist < 0.4:
                base_servo = 105
                reason += " + right wall close → softer"
        
        self.get_logger().info(
            f"🎮 Servo Strategy: {aggression} ({base_servo}°) - {reason}"
        )
        
        return base_servo

    def _on_scan(self, msg: LaserScan):
        self.last_scan_msg = msg
        self.last_scan_time = self.get_clock().now()

        degs = self._build_signed_deg_array(msg)
        rngs = np.array(msg.ranges, dtype=np.float32)
        m_front = self._sector_mask_signed(degs, *self.FRONT_DEG)
        self.last_front_dist = self._robust_sector_distance(rngs, m_front)

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

    def _get_left_wall_distance(self):
        msg = self.last_scan_msg
        if msg is None:
            return None
        ranges = np.array(msg.ranges)
        angle_increment = msg.angle_increment
        total_points = len(ranges)
        left_start_rad = np.radians(70)
        left_end_rad = np.radians(90)
        left_start_idx = int((left_start_rad + np.pi/2) / angle_increment)
        left_end_idx = int((left_end_rad + np.pi/2) / angle_increment)
        left_start_idx = max(0, min(total_points - 1, left_start_idx))
        left_end_idx = max(0, min(total_points, left_end_idx))
        left_ranges = ranges[left_start_idx:left_end_idx]
        valid = left_ranges[np.isfinite(left_ranges)]
        valid = valid[(valid > 0) & (valid < 2.0)]
        if valid.size > 0:
            return float(np.median(valid))
        return None
    
    def _check_obstacle_clearance(self, obstacle_width):
        msg = self.last_scan_msg
        if msg is None or obstacle_width is None:
            return True, None, None, "No LiDAR data"
        
        left_dist, right_dist = self._get_side_distances()
        
        front_dist, _, _ = self._get_front_width_and_distance()
        
        if front_dist is None:
            return True, None, None, "No obstacle detected"
        
        car_width = self.CAR_WIDTH
        min_clearance = self.MIN_SIDE_CLEARANCE
        
        if left_dist is not None:
            left_gap = left_dist - (obstacle_width / 2.0)
        else:
            left_gap = None
        
        if right_dist is not None:
            right_gap = right_dist - (obstacle_width / 2.0)
        else:
            right_gap = None
        
        can_pass_left = (left_gap is not None and left_gap >= car_width + min_clearance)
        can_pass_right = (right_gap is not None and right_gap >= car_width + min_clearance)
        
        can_pass = can_pass_left or can_pass_right
        
        if not can_pass:
            if left_gap is not None and right_gap is not None:
                reason = f"Both sides too narrow (L:{left_gap:.2f}m, R:{right_gap:.2f}m < {car_width + min_clearance:.2f}m)"
            elif left_gap is not None:
                reason = f"Left gap too narrow ({left_gap:.2f}m < {car_width + min_clearance:.2f}m), no right wall data"
            elif right_gap is not None:
                reason = f"Right gap too narrow ({right_gap:.2f}m < {car_width + min_clearance:.2f}m), no left wall data"
            else:
                reason = "No wall data to calculate gaps"
        else:
            pass_side = []
            if can_pass_left:
                pass_side.append(f"LEFT ({left_gap:.2f}m)")
            if can_pass_right:
                pass_side.append(f"RIGHT ({right_gap:.2f}m)")
            reason = f"Can pass via {' or '.join(pass_side)}"
        
        return can_pass, left_gap, right_gap, reason

    def _get_front_width_and_distance(self, angle_deg=15.0, max_range=None):
        msg = self.last_scan_msg
        if msg is None:
            return None, None, False

        ranges = np.array(msg.ranges)
        angle_increment = msg.angle_increment
        total_points = len(ranges)
        
        if max_range is None:
            max_range = self.RANGE_MAX

        front_angle_rad = np.radians(angle_deg)
        center_index = total_points // 2
        angle_range_indices = int(front_angle_rad / angle_increment)
        start_idx = max(0, center_index - angle_range_indices)
        end_idx = min(total_points, center_index + angle_range_indices)

        front_ranges = ranges[start_idx:end_idx]
        valid_mask = np.isfinite(front_ranges) & (front_ranges >= self.RANGE_MIN) & (front_ranges < max_range)
        valid_front = front_ranges[valid_mask]
        
        if valid_front.size == 0:
            return None, None, False
        
        front_distance = float(np.min(valid_front))

        distance_tolerance = 0.06
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
            
            obstacle_type = 'OBSTACLE (narrow)' if is_obstacle else 'WALL (wide)'
            self.get_logger().info(
                f"📏 LiDAR width: dist={front_distance:.3f}m, pts={max_consecutive}, "
                f"span={np.degrees(angle_span):.1f}°, width={est_width:.3f}m, "
                f"thresh={self.WALL_WIDTH_THRESHOLD:.3f}m → {obstacle_type}"
            )
            
            return front_distance, est_width, is_obstacle
        
        return front_distance, None, False

    def _front_distance_and_is_wall(self):
        dist, _, is_obstacle = self._get_front_width_and_distance(
            angle_deg=self.FRONT_SCAN_ANGLE_RANGE_DEG / 2.0,
            max_range=self.FRONT_MAX_RANGE
        )
        if dist is None:
            return None, False
        is_wall = not is_obstacle
        return dist, is_wall

    def _get_obstacle_width_and_distance(self):
        return self._get_front_width_and_distance(
            angle_deg=15.0,
            max_range=self.RANGE_MAX
        )

    def _is_corner(self):
        current_time = time.time()
        
        # Check corner detection cooldown FIRST
        if current_time < self.corner_detection_cooldown:
            remaining = self.corner_detection_cooldown - current_time
            self.get_logger().info(
                f"🚫⏳ Corner detection on cooldown: {remaining:.2f}s remaining"
            )
            return False
        
        # LiDAR-based corner detection
        front_dist, is_wall = self._front_distance_and_is_wall()
        if front_dist is None:
            self.get_logger().info("🚫 Corner check: front distance is None (no LiDAR data)")
            return False
        
        if not is_wall:
            self.get_logger().info(
                f"🚫 Not corner - narrow object detected (pillar?): front={front_dist:.3f}m, "
                f"width < {self.WALL_WIDTH_THRESHOLD:.3f}m"
            )
            return False
        
        # Get vision detections for pillar-based corner detection
        has_red_in_left_zone = False
        has_green_in_left_zone = False
        green_pillar_close = False
        try:
            with self.frame_lock:
                detections = self.current_detections.copy() if hasattr(self, 'current_detections') else []
            has_red_in_left_zone = any(
                d['color'] == 'red' and d['zone'] == 'right' and d['distance'] <= 1.5
                for d in detections
            )
            has_green_in_left_zone = any(
                d['color'] == 'green' and d['zone'] == 'right' and d['distance'] <= 1.5
                for d in detections
            )
        except:
            pass
        
        if has_red_in_left_zone:
            in_window = (self.FRONT_CORNER_WINDOW[0] <= front_dist <= 0.45)
            if not in_window:
                return False
            
            left_dist = self._get_left_wall_distance()
            if left_dist is None:
                return False
            
            is_corner = left_dist <= 0.70
            if is_corner:
                self.corner_detection_cooldown = time.time() + self.CORNER_DETECTION_COOLDOWN  # Set general cooldown
                self.get_logger().info(
                    f"🔴 CORNER DETECTED (red pillar near outer wall): "
                    f"front={front_dist:.3f}m (WALL verified), right={right_dist:.3f}m → TRIGGERING REVERSE EARLY"
                )
                self.get_logger().info(
                    f"🔒 ALL corner detection LOCKED for {self.CORNER_DETECTION_COOLDOWN:.1f} seconds"
                )
            return is_corner
        
        # 🆕 NEW: Green pillar + front wall combo detection
        # If there's a green pillar in right zone AND a wall close ahead, it's likely a corner
        # This helps when robot is too close to inner wall after avoiding last green pillar
        if has_green_in_left_zone:
            # More lenient front distance check (0.30-0.60m) since we may be close to inner wall
            in_green_combo_window = (0.30 <= front_dist <= 0.60)
            if in_green_combo_window:
                # Don't require right wall detection - we're too close to inner (left) wall to see it clearly
                self.corner_detection_cooldown = time.time() + self.CORNER_DETECTION_COOLDOWN
                self.get_logger().info(
                    f"🟢 CORNER DETECTED (green pillar + front wall combo): "
                    f"green pillar in RIGHT zone + front wall at {front_dist:.3f}m → TRIGGERING REVERSE"
                )
                self.get_logger().info(
                    f"💡 This detection helps when robot is near inner wall after last green pillar"
                )
                self.get_logger().info(
                    f"🔒 ALL corner detection LOCKED for {self.CORNER_DETECTION_COOLDOWN:.1f} seconds"
                )
                return True
            else:
                self.get_logger().info(
                    f"🟢 Green pillar in right zone detected, but front wall too far: {front_dist:.3f}m > 0.60m (not corner yet)"
                )
        
        # Normal LiDAR-only corner detection (fallback)
        else:
            in_window = (self.FRONT_CORNER_WINDOW[0] <= front_dist <= self.FRONT_CORNER_WINDOW[1])
            if not in_window:
                if front_dist < self.FRONT_CORNER_WINDOW[0]:
                    self.get_logger().info(
                        f"⚠️ Front WALL too close: {front_dist:.3f}m < {self.FRONT_CORNER_WINDOW[0]:.2f}m "
                        f"(already past corner window!)"
                    )
                return False
            
            left_dist = self._get_left_wall_distance()
            if left_dist is None:
                self.get_logger().info(
                    f"⚠️ Front wall in range ({front_dist:.3f}m) but no right wall detected"
                )
                return False
            
            is_corner = left_dist <= self.LEFT_NEAR_THRESHOLD
            if is_corner:
                self.corner_detection_cooldown = time.time() + self.CORNER_DETECTION_COOLDOWN  # Set general cooldown
                self.get_logger().info(
                    f"🏁 CORNER DETECTED: front={front_dist:.3f}m (WALL verified), "
                    f"left={left_dist:.3f}m ≤ {self.LEFT_NEAR_THRESHOLD:.2f}m → TRIGGERING REVERSE"
                )
                self.get_logger().info(
                    f"🔒 ALL corner detection LOCKED for {self.CORNER_DETECTION_COOLDOWN:.1f} seconds"
                )
            else:
                self.get_logger().info(
                    f"🚧 Front wall detected ({front_dist:.3f}m) but right wall too far: "
                    f"right={left_dist:.3f}m > {self.LEFT_NEAR_THRESHOLD:.2f}m (not a corner)"
                )
            return is_corner
    
    def _is_approaching_corner(self):
        front_dist, is_wall = self._front_distance_and_is_wall()
        if front_dist is None:
            return False
        
        if not is_wall:
            return False
        
        in_approach_window = (0.35 < front_dist <= 0.60)
        
        if not in_approach_window:
            return False
        
        left_dist = self._get_left_wall_distance()
        if left_dist is None:
            return False
        
        approaching = left_dist <= 0.80
        
        if approaching:
            self.get_logger().info(
                f"🚦 Approaching corner detected: front={front_dist:.3f}m (WALL), "
                f"left={left_dist:.3f}m → DISABLING side wall corrections"
            )
        
        return approaching

    def _detect_lidar_obstacle(self):
        dist, width, is_obstacle = self._get_obstacle_width_and_distance()
        
        if dist is None or width is None:
            return None, None, False
        
        if dist <= self.OBSTACLE_DETECT_DISTANCE and is_obstacle:
            if 0.8 <= dist <= 1.2 and width is not None and width < self.WALL_WIDTH_THRESHOLD:
                self.get_logger().warning(
                    f"⚠️ LiDAR BLIND SPOT WARNING: dist={dist:.3f}m, width={width:.3f}m → "
                    f"Object may be MUCH CLOSER than {dist:.3f}m! (LiDAR min range = {self.RANGE_MIN}m, "
                    f"readings below this are invalid)"
                )
            else:
                self.get_logger().info(
                    f"✅ LiDAR WIDTH: dist={dist:.3f}m, width={width:.3f}m → NARROW (< {self.WALL_WIDTH_THRESHOLD:.3f}m) = PILLAR"
                )
            return dist, width, True
        elif dist <= self.OBSTACLE_DETECT_DISTANCE and not is_obstacle:
            self.get_logger().info(
                f"🧱 LiDAR WIDTH: dist={dist:.3f}m, width={width:.3f}m → WIDE (≥ {self.WALL_WIDTH_THRESHOLD:.3f}m) = WALL"
            )
        
        return dist, width, False

    def _is_wall_too_close(self, left_dist, right_dist):
        left_too_close = (left_dist is not None and left_dist < self.PANIC_WALL_DISTANCE)
        right_too_close = (right_dist is not None and right_dist < self.PANIC_WALL_DISTANCE)
        return left_too_close, right_too_close
    
    def _is_wall_getting_close(self, left_dist, right_dist):
        left_getting_close = (left_dist is not None and 
                             self.PANIC_WALL_DISTANCE < left_dist < self.WARNING_WALL_DISTANCE)
        right_getting_close = (right_dist is not None and 
                              self.PANIC_WALL_DISTANCE < right_dist < self.WARNING_WALL_DISTANCE)
        return left_getting_close, right_getting_close
    
    def _calculate_panic_steering(self, left_too_close, right_too_close, left_dist, right_dist):
        if left_too_close and right_too_close:
            if left_dist is not None and right_dist is not None:
                if left_dist <= right_dist:
                    return self.PANIC_SERVO_HARD_RIGHT
                else:
                    return self.PANIC_SERVO_HARD_LEFT
            else:
                return self.PANIC_SERVO_HARD_LEFT
        
        elif left_too_close:
            return self.PANIC_SERVO_HARD_RIGHT
        
        else:
            return self.PANIC_SERVO_HARD_LEFT
    
    def _calculate_warning_steering(self, left_getting_close, right_getting_close, left_dist, right_dist):
        if left_getting_close and right_getting_close:
            if left_dist is not None and right_dist is not None:
                if left_dist <= right_dist:
                    return self.WARNING_SERVO_RIGHT
                else:
                    return self.WARNING_SERVO_LEFT
            else:
                return self.servo_center
        
        elif left_getting_close:
            return self.WARNING_SERVO_RIGHT
        
        else:
            return self.WARNING_SERVO_LEFT
    
    def _blend_steering_with_pillar(self, wall_steering, blend_ratio=0.5):
        if not self.pillar_avoidance_active:
            return wall_steering
        
        blended = (wall_steering * blend_ratio) + (self.pillar_avoid_direction * (1 - blend_ratio))
        return blended
    
    def _log_panic_situation(self, left_too_close, right_too_close, left_dist, right_dist, final_servo, speed):
        if left_too_close and not right_too_close:
            self.get_logger().warning(
                f"🚨 PANIC LEFT! Wall at {left_dist:.2f}m < {self.PANIC_WALL_DISTANCE:.2f}m "
                f"→ HARD RIGHT to {final_servo:.0f}°, speed={speed}"
            )
        elif right_too_close and not left_too_close:
            self.get_logger().warning(
                f"🚨 PANIC RIGHT! Wall at {right_dist:.2f}m < {self.PANIC_WALL_DISTANCE:.2f}m "
                f"→ HARD LEFT to {final_servo:.0f}°, speed={speed}"
            )
        else:
            left_str = f"{left_dist:.2f}m" if left_dist else "N/A"
            right_str = f"{right_dist:.2f}m" if right_dist else "N/A"
            self.get_logger().warning(
                f"🚨 PANIC BOTH WALLS! L={left_str}, R={right_str} "
                f"→ servo={final_servo:.0f}°, speed={speed}"
            )
    
    def _log_warning_situation(self, left_getting_close, right_getting_close, left_dist, right_dist, final_servo):
        if left_getting_close and not right_getting_close:
            self.get_logger().warning(
                f"⚠️ WARNING LEFT! Wall at {left_dist:.2f}m → moderate RIGHT to {final_servo:.0f}°"
            )
        elif right_getting_close and not left_getting_close:
            self.get_logger().warning(
                f"⚠️ WARNING RIGHT! Wall at {right_dist:.2f}m → moderate LEFT to {final_servo:.0f}°"
            )
        else:
            self.get_logger().warning(
                f"⚠️ WARNING BOTH! L={left_dist:.2f}m, R={right_dist:.2f}m → servo={final_servo:.0f}°"
            )
    
    def _control_loop(self):
        if self.performing_backoff:
            self._handle_backoff()
            return

        if self._is_corner():
            if self.pillar_avoidance_active:
                self.get_logger().info("🛑 Disabling pillar avoidance - CORNER takes priority!")
                self.pillar_avoidance_active = False
                self.pillar_avoid_direction = self.servo_center
                self.tracked_pillar = None
            self._start_backoff()
            return

        approaching_corner = self._is_approaching_corner()
        
        left_dist, right_dist = self._get_side_distances()
        
        left_too_close, right_too_close = self._is_wall_too_close(left_dist, right_dist)
        any_wall_too_close = left_too_close or right_too_close
        
        if any_wall_too_close and not approaching_corner:
            if self.pillar_avoidance_active:
                self.get_logger().warning(
                    "🚨 PANIC WALL OVERRIDE - Temporarily disabling pillar avoidance for wall safety!"
                )
                self.pillar_avoidance_active = False
                self.pillar_avoid_direction = self.servo_center
            
            wall_steering = self._calculate_panic_steering(left_too_close, right_too_close, left_dist, right_dist)
            
            final_servo = self._blend_steering_with_pillar(wall_steering, blend_ratio=0.5)
            
            panic_speed = int(max(self.min_speed, self.base_speed * 0.5))
            final_servo = max(self.servo_min, min(self.servo_max, final_servo))
            
            self._log_panic_situation(left_too_close, right_too_close, left_dist, right_dist, final_servo, panic_speed)
            if self.pillar_avoidance_active:
                self.get_logger().warning(
                    f"   └─ Blended: wall={wall_steering:.0f}° + pillar={self.pillar_avoid_direction:.0f}° "
                    f"→ final={final_servo:.0f}°"
                )
            
            with self._state_lock:
                self.last_servo_deg = float(final_servo)
                self.last_speed_cps = int(panic_speed)
            self._send_can(speed=self.last_speed_cps, servo_deg=self.last_servo_deg)
            return
        
        elif any_wall_too_close and approaching_corner:
            left_str = f"{left_dist:.2f}m" if left_dist else 'N/A'
            right_str = f"{right_dist:.2f}m" if right_dist else 'N/A'
            self.get_logger().info(
                f"⚠️ Wall too close BUT approaching corner - corner takes priority (L={left_str}, R={right_str})"
            )
        
        left_getting_close, right_getting_close = self._is_wall_getting_close(left_dist, right_dist)
        any_wall_getting_close = left_getting_close or right_getting_close
        
        if any_wall_getting_close and not approaching_corner:
            wall_steering = self._calculate_warning_steering(left_getting_close, right_getting_close, left_dist, right_dist)
            
            final_servo = self._blend_steering_with_pillar(wall_steering, blend_ratio=0.7)
            
            if self.pillar_avoidance_active:
                self.get_logger().warning(
                    f"⚠️ WARNING + PILLAR: wall={wall_steering:.0f}° (70%) + "
                    f"pillar={self.pillar_avoid_direction:.0f}° (30%) → {final_servo:.0f}°"
                )
            else:
                warning_speed = int(self.base_speed * 0.8)
                final_servo = max(self.servo_min, min(self.servo_max, final_servo))
                
                self._log_warning_situation(left_getting_close, right_getting_close, left_dist, right_dist, final_servo)
                with self._state_lock:
                    self.last_servo_deg = float(final_servo)
                    self.last_speed_cps = int(warning_speed)
                self._send_can(speed=self.last_speed_cps, servo_deg=self.last_servo_deg)
                return

        servo = self.servo_center
        speed = self.base_speed
        
        if self.vision_enabled and not self.vision_ready:
            elapsed = time.time() - self.get_clock().now().nanoseconds / 1e9
            if elapsed > 2.0:
                self.get_logger().warning("⏳ Vision not ready yet - waiting for first frame...")
            with self._state_lock:
                self.last_servo_deg = float(servo)
                self.last_speed_cps = int(speed)
            self._send_can(speed=self.last_speed_cps, servo_deg=self.last_servo_deg)
            return

        if self.vision_enabled and self.vision_ready and self.pillar_avoidance_active:
            servo = float(self.pillar_avoid_direction)
            speed = int(self.base_speed * 0.7)
            
            direction = "LEFT ⬅️" if servo < 90 else "RIGHT ➡️" if servo > 90 else "CENTER"
            self.get_logger().info(f"🎯 PILLAR AVOIDANCE ACTIVE: {direction} servo={servo:.1f}°, speed={speed}")

            elapsed_time = time.time() - self.pillar_avoidance_start_time
            lidar_timeout_check_dist, _, lidar_timeout_check_obstacle = self._detect_lidar_obstacle()
            
            lidar_clear = not lidar_timeout_check_obstacle or lidar_timeout_check_dist is None or lidar_timeout_check_dist > 1.2
            timeout_reached = elapsed_time > 4.0
            
            activation_dist = self.tracked_pillar.get('activation_distance', float('inf')) if self.tracked_pillar else float('inf')
            distance_increased = (lidar_timeout_check_dist is not None and 
                                activation_dist != float('inf') and 
                                lidar_timeout_check_dist > activation_dist + 0.5)
            
            pillar_lost = self.tracked_pillar is None
            
            camera_and_lidar_agree_passed = False
            if pillar_lost and (lidar_clear or distance_increased):
                camera_and_lidar_agree_passed = True
                self.get_logger().info("✅ Both camera and LiDAR confirm pillar passed")
            
            if camera_and_lidar_agree_passed or timeout_reached:
                self.pillar_avoidance_active = False
                self.pillar_avoid_direction = self.servo_center
                self.tracked_pillar = None
                self.pillar_avoidance_cooldown = time.time() + 0.5
                
                reason = "Camera+LiDAR agree passed" if camera_and_lidar_agree_passed else "Timeout"
                self.get_logger().info(f"✅ Pillar avoidance complete - deactivating. Reason: {reason}, ready for next obstacle")

        now_time = time.time()
        left_dist, right_dist = self._get_side_distances()
        left_close = (left_dist is not None and left_dist < self.SIDE_NEAR_THRESHOLD)
        right_close = (right_dist is not None and right_dist < self.SIDE_NEAR_THRESHOLD)
        
        in_initial_avoidance_phase = (
            self.pillar_avoidance_active and 
            now_time <= self.side_wall_disabled_until
        )
        
        if approaching_corner and (left_close or right_close):
            left_str = f"{left_dist:.2f}m" if left_dist else 'N/A'
            right_str = f"{right_dist:.2f}m" if right_dist else 'N/A'
            self.get_logger().info(
                f"🚦 APPROACHING CORNER - ignoring side wall corrections (L={left_str}, R={right_str})"
            )
        elif in_initial_avoidance_phase and (left_close or right_close):
            left_str = f"{left_dist:.2f}m" if left_dist else 'N/A'
            right_str = f"{right_dist:.2f}m" if right_dist else 'N/A'
            self.get_logger().info(
                f"🎯 PILLAR AVOIDANCE INIT - ignoring walls to establish direction first (L={left_str}, R={right_str})"
            )
        elif (left_close or right_close):
            wall_correction = 0.0
            log_msg = ""
            
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
            
            if self.pillar_avoidance_active:
                pillar_direction = "LEFT" if self.pillar_avoid_direction < 90 else "RIGHT"
                
                if (left_close and pillar_direction == "RIGHT") or (right_close and pillar_direction == "LEFT"):
                    if wall_correction > 2.0:
                        wall_correction = 2.0
                    elif wall_correction < -2.0:
                        wall_correction = -2.0
                    
                    servo = float(servo) + wall_correction
                    self.get_logger().warning(
                        f"🚨 CONFLICT: {log_msg} but pillar wants {pillar_direction} → "
                        f"REDUCED wall correction: {wall_correction:+.1f}° → final={servo:.1f}°"
                    )
                else:
                    if wall_correction > 2.0:
                        wall_correction = 2.0
                    elif wall_correction < -2.0:
                        wall_correction = -2.0
                    
                    servo = float(servo) + wall_correction
                    self.get_logger().info(
                        f"{log_msg} [ASSIST: +{wall_correction:+.1f}° to pillar {self.pillar_avoid_direction:.1f}° → final={servo:.1f}°]"
                    )
            else:
                servo = float(servo) + wall_correction
                self.get_logger().info(f"{log_msg} → servo={servo:.1f}°")

        servo = max(self.servo_min, min(self.servo_max, servo))
        
        target_servo = float(max(self.servo_min, min(self.servo_max, servo)))
        try:
            prev_servo = float(self.last_servo_deg)
        except Exception:
            prev_servo = float(self.servo_center)
        max_step = float(self.SERVO_MAX_SLEW_DEG_PER_SEC) / float(self.send_rate_hz)
        delta = target_servo - prev_servo
        if delta > max_step:
            servo = prev_servo + max_step
        elif delta < -max_step:
            servo = prev_servo - max_step
        else:
            servo = target_servo

        if self.pillar_avoidance_active or left_close or right_close:
            self.get_logger().info(f"📤 FINAL COMMAND: servo={servo:.1f}°, speed={speed}")
        with self._state_lock:
            self.last_servo_deg = float(servo)
            self.last_speed_cps = int(speed)
        self._send_can(speed=self.last_speed_cps, servo_deg=self.last_servo_deg)

    def _start_backoff(self):
        self.performing_backoff = True
        self.backoff_stage = 1
        self.backoff_start_time = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().info(
            f"↩️ Corner detected → Stage1: pre-turn RIGHT {self.PRETURN_SERVO}° for {self.PRETURN_DURATION:.1f}s (avoid inside wall), "
            f"then Stage2: reverse {self.BACKOFF_DURATION:.1f}s (servo={self.BACKOFF_SERVO}°)."
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
            left_dist = self._get_left_wall_distance()
            servo = self.BACKOFF_SERVO
            if left_dist is not None:
                error = left_dist - self.TARGET_LEFT_DISTANCE
                servo += self.REVERSE_KP * error
                servo = max(35, min(145, servo))
                self.get_logger().info(
                    f"↩️ Backoff Stage2: Left={left_dist:.3f}m, Servo={servo:.1f}°, Speed={self.BACKOFF_SPEED}, t={elapsed:.2f}s"
                )
            else:
                self.get_logger().info(
                    f"↩️ Backoff Stage2 (no left wall): Servo={servo}°, Speed={self.BACKOFF_SPEED}, t={elapsed:.2f}s"
                )
            self._send_can(speed=self.BACKOFF_SPEED, servo_deg=servo)
            if elapsed >= self.BACKOFF_DURATION:
                self.backoff_stage = 3
                self.backoff_start_time = now
        elif self.backoff_stage == 3:
            servo = self.CENTER_SERVO
            speed = self.base_speed
            self._send_can(speed=speed, servo_deg=servo)
            self.get_logger().info(f"↩️ Backoff Stage3: Servo={servo}°, Speed={speed}, t={elapsed:.2f}s")
            if elapsed >= self.CENTER_DURATION:
                self.performing_backoff = False
                self.backoff_stage = 0
                with self._state_lock:
                    self.last_servo_deg = self.servo_center
                    self.last_speed_cps = self.base_speed
                self.get_logger().info("✅ Backoff complete — resuming normal operation.")

    def _send_can(self, speed: int, servo_deg: float):
        servo_raw = int(round(servo_deg * 100.0))
        data = struct.pack(">ii", int(speed), int(servo_raw))
        
        if self.bus is None:
            try:
                self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=250000)
                self.get_logger().info("CAN bus reinitialized")
            except Exception as e:
                self.get_logger().error(f"Cannot initialize CAN bus: {e}")
                return
            
        try:
            msg = can.Message(
                arbitration_id=0x101, 
                data=data, 
                is_extended_id=False,
                is_remote_frame=False,
                dlc=len(data)
            )
            
            with self._can_send_lock:
                try:
                    self.bus.send(msg, timeout=0.02)
                    if self.send_rate_hz > 30:
                        time.sleep(0.001)
                        
                except can.CanError as e:
                    self.get_logger().error(f"CAN send failed: {e}")
                    self._can_send_fail_count += 1
                    
                    if self._can_send_fail_count > 5:
                        self.get_logger().warning("Multiple CAN failures - resetting bus")
                        self._reset_can_bus()
                        
        except Exception as e:
            self.get_logger().warning(f"CAN message preparation error: {e}")

    def _reset_can_bus(self):
        try:
            if self.bus:
                self.bus.shutdown()
                time.sleep(0.1)
            self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=250000)
            self._can_send_fail_count = 0
            self.get_logger().info("✅ CAN bus reset successfully")
        except Exception as e:
            self.get_logger().error(f"CAN bus reset failed: {e}")
            self.bus = None

    def _get_side_distances(self):
        msg = self.last_scan_msg
        if msg is None:
            return None, None
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
        return d_left, d_right

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
        self.performing_backoff = False
        self.backoff_stage = 0
        self.pillar_avoidance_active = False
        self.pillar_avoid_direction = self.servo_center
        self.tracked_pillar = None
        self.pillar_avoidance_cooldown = 0.0
        self.side_wall_disabled_until = 0.0
        self.obstacle_queue = []
        self.corner_detection_cooldown = 0.0  # Reset corner cooldown
        with self._state_lock:
            self.last_servo_deg = self.servo_center
            self.last_speed_cps = self.base_speed

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
        try:
            if self.pipeline is not None:
                self.pipeline.stop()
        except Exception:
            pass
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
