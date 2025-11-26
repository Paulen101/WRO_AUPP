#!/usr/bin/env python3
import struct
import threading
import time
import numpy as np
import can
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import math

# ============================================================================
# CONFIGURATION CONSTANTS
# ============================================================================

# CAN Communication
CAN_INTERFACE = 'socketcan'
CAN_CHANNEL = 'can0'
CAN_BITRATE = 250000
CAN_TX_ID = 0x101               # Motor command ID
CAN_RX_ID = 0x200               # Encoder feedback ID
SEND_RATE_HZ = 20.0             # CAN message send rate

# Servo Steering
SERVO_CENTER = 90.0             # Center position (degrees)
SERVO_MIN = 30.0                # Minimum servo angle (degrees)
SERVO_MAX = 160.0               # Maximum servo angle (degrees)
STEER_GAIN = 60.0               # Steering sensitivity (degrees per radian)
ANG_LIMIT = 2.0                 # Maximum angular command limit (radians)

# Wall Following Parameters
TARGET_DISTANCE = 0.6           # Target distance from center (meters)
MIN_WALL_DISTANCE = 0.25        # Emergency distance - hard steer away (meters)

# PID Controller Gains
KP = 2.5                        # Proportional gain
KI = 0.05                       # Integral gain
KD = 0.8                        # Derivative gain
INTEGRAL_LIMIT = 5.0            # Anti-windup limit

# Safety margins
SAFETY_MARGIN = 0.15            # Extra distance to maintain from walls (meters)
PANIC_DISTANCE = 0.20           # Distance to trigger emergency avoidance (meters)
PANIC_STEER_STRENGTH = 2.0      # Emergency steering strength (radians)

# Smoothing and filtering
ERROR_SMOOTHING_ALPHA = 0.3     # Exponential smoothing for error (0-1, lower = more smooth)
MIN_CORRECTION_THRESHOLD = 0.05 # Minimum error to apply correction (meters)

# Speed Control
MAX_SPEED = 1000                 # Maximum cruising speed (0-2000)
MIN_SPEED = 300                 # Minimum speed during slowdown
SLOW_SPEED = 500                # Speed when correcting heavily
SPEED_REDUCTION_FACTOR = 0.5    # How much to slow down when steering (0-1)
STEERING_THRESHOLD = 0.3        # Steering angle to start reducing speed (radians)

# System
SCAN_TIMEOUT = 2.0              # Watchdog timeout for LiDAR (seconds)

# LiDAR Processing Constants (for LidarProcessor)
RANGE_MIN = 0.08                # Minimum valid LiDAR range (meters)
RANGE_MAX = 4.0                 # Maximum valid LiDAR range (meters)
MIN_HITS = 5                    # Minimum points required for valid sector reading
K_FRACT = 0.3                   # Fraction of closest points to use for robust distance

# LiDAR Sector Angles (degrees) - wider sectors for robust wall detection
LEFT_SECTOR = (+45.0, +135.0)    # Left side detection zone (90° wide)
RIGHT_SECTOR = (-135.0, -45.0)   # Right side detection zone (90° wide)
OBSTACLE_FRONT = (-15.0, +15.0) # Front obstacle detection zone


# ============================================================================
# LIDAR PROCESSOR CLASS
# ============================================================================

class LidarProcessor:
    """LiDAR data processing utilities"""
    
    @staticmethod
    def process_scan(msg):
        """Extract distances from laser scan message
        
        Args:
            msg: LaserScan message from ROS2
            
        Returns:
            dict: Distances for 'front', 'left', 'right' sectors
        """
        n = len(msg.ranges)
        ang_rads = msg.angle_min + np.arange(n) * msg.angle_increment
        degs = np.degrees(ang_rads)
        rngs = np.array(msg.ranges, dtype=np.float32)
        
        # Calculate sector distances
        sectors = {
            'front': LidarProcessor._sector_dist(rngs, degs, *OBSTACLE_FRONT),
            'left': LidarProcessor._sector_dist(rngs, degs, *LEFT_SECTOR),
            'right': LidarProcessor._sector_dist(rngs, degs, *RIGHT_SECTOR)
        }
        
        # Fallback for NaN front
        if np.isnan(sectors['front']):
            sectors['front'] = LidarProcessor._sector_dist(rngs, degs, -20.0, 20.0)
        
        return sectors
    
    @staticmethod
    def _sector_dist(ranges, degs, lo, hi):
        """Calculate robust distance for angle sector
        
        Uses the mean of the closest K_FRACT fraction of points
        to filter out noise and outliers.
        
        Args:
            ranges: Array of range measurements
            degs: Array of angles in degrees
            lo: Lower angle bound
            hi: Upper angle bound
            
        Returns:
            float: Robust distance estimate or NaN if insufficient data
        """
        mask = (degs >= lo) & (degs <= hi) if lo <= hi else (degs >= lo) | (degs <= hi)
        vals = ranges[mask]
        vals = vals[np.isfinite(vals) & (vals >= RANGE_MIN) & (vals <= RANGE_MAX)]
        
        if vals.size < MIN_HITS:
            return float('nan')
        
        vals.sort()
        k = max(5, int(K_FRACT * vals.size))
        return float(np.mean(vals[:k]))
    
    @staticmethod
    def normalize_angle(target, current):
        """Normalize angle difference to [-180, 180]
        
        Args:
            target: Target angle (degrees)
            current: Current angle (degrees)
            
        Returns:
            float: Normalized angle difference
        """
        diff = target - current
        while diff > 180:
            diff -= 360
        while diff < -180:
            diff += 360
        return diff


# ============================================================================
# HELPER FUNCTIONS
# ============================================================================
def quaternion_to_yaw_degrees(x, y, z, w):
    """Convert quaternion to yaw angle in degrees (0-360 range)."""
    # Calculate yaw from quaternion
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw_rad = math.atan2(siny_cosp, cosy_cosp)
    # Convert to degrees and normalize to 0-360
    yaw_deg = math.degrees(yaw_rad)
    # Convert from -180..180 to 0..360
    if yaw_deg < 0:
        yaw_deg += 360.0
    return yaw_deg


def unwrap_angle_deg(current, previous):
    """Unwrap angle to handle 360° wraparound for rotation tracking.
    
    Args:
        current: Current angle in degrees (0-360)
        previous: Previous angle in degrees (0-360)
        
    Returns:
        tuple: (unwrapped_current, delta) where delta is the change accounting for wraparound
    """
    if previous is None:
        return current, 0.0
    
    # Calculate the raw difference
    delta = current - previous
    
    # Handle wraparound
    if delta > 180:
        delta -= 360
    elif delta < -180:
        delta += 360
    
    # Accumulate the unwrapped angle
    unwrapped = previous + delta
    
    return unwrapped, delta


class WallFollowController(Node):
    """ROS2 node for wall-following behavior"""
    
    def __init__(self):
        super().__init__('wall_follow_controller')
        
        # Declare parameters
        self._declare_params()
        self.params = self._load_params()
        
        # State variables
        self.state_lock = threading.Lock()
        self.last_servo = SERVO_CENTER
        self.last_speed = 0
        self.last_scan_time = None
        self.running = True
        
        # PID controller state
        self.prev_error = 0.0
        self.integral = 0.0
        self.smoothed_error = 0.0
        
        # IMU-based lap tracking state
        self.passed_90 = False
        self.passed_180 = False
        self.passed_270 = False
        self.rotation_count = 0
        self.robot_stopped = False
        self.slowdown_done = False  # Track if we've applied 3rd lap slowdown
        self.all_sectors_passed = False  # Track when all sectors have been passed
        self.last_yaw = None  # Track previous yaw for boundary crossing detection
        self.yaw_offset = None  # Initialize yaw offset on first IMU reading
        self.total_rotation = 0.0  # Track total rotation angle (can be positive or negative)
        self.base_max_speed = MAX_SPEED  # Store original max speed
        
        # First lap boost control
        self.first90_active = False  # Currently in the 0-90° boost window
        self.first90_done = False    # Boost already used once
        
        # IMU logging counter
        self._imu_log_counter = 0
        
        # Initialize CAN
        self._init_can()
        
        # Create ROS interfaces
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )
        self.create_subscription(LaserScan, '/scan_180', self._on_scan, qos)
        self.create_subscription(Imu, '/bno055/imu', self._imu_callback, 10)
        self.create_timer(1.0 / SEND_RATE_HZ, self._send_can)
        self.create_timer(1.0, self._check_watchdog)
        
        self.get_logger().info("✅ Wall Following Controller ready")
        self.get_logger().info(f"   Target wall distance: {self.params['target_distance']:.2f}m")
        self.get_logger().info(f"   Mode: Centering between left and right walls")
        self.get_logger().info(f"   IMU lap counting: 3 laps with boost on first lap (works for both rotation directions)")
        self.get_logger().info(f"   Speed profile: boost@0-90°(lap1) → slowdown@270°(lap3) → stop after lap3")
        self.get_logger().info(f"   Subscribed to /bno055/imu for lap tracking (with zero initialization, bidirectional)")
    
    def _declare_params(self):
        """Declare ROS2 parameters"""
        self.declare_parameter('target_distance', TARGET_DISTANCE)
        self.declare_parameter('kp', KP)
        self.declare_parameter('ki', KI)
        self.declare_parameter('kd', KD)
        self.declare_parameter('max_speed', MAX_SPEED)
        self.declare_parameter('min_speed', MIN_SPEED)
        self.declare_parameter('slow_speed', SLOW_SPEED)
        self.declare_parameter('min_wall_distance', MIN_WALL_DISTANCE)
        self.declare_parameter('safety_margin', SAFETY_MARGIN)
    
    def _load_params(self):
        """Load parameters into dict"""
        return {
            'target_distance': self.get_parameter('target_distance').value,
            'kp': self.get_parameter('kp').value,
            'ki': self.get_parameter('ki').value,
            'kd': self.get_parameter('kd').value,
            'max_speed': self.get_parameter('max_speed').value,
            'min_speed': self.get_parameter('min_speed').value,
            'slow_speed': self.get_parameter('slow_speed').value,
            'min_wall_distance': self.get_parameter('min_wall_distance').value,
            'safety_margin': self.get_parameter('safety_margin').value
        }
    
    def _init_can(self):
        """Initialize CAN bus"""
        self.can_lock = threading.Lock()
        self.rx_stop = threading.Event()
        try:
            self.bus = can.interface.Bus(
                interface=CAN_INTERFACE,
                channel=CAN_CHANNEL,
                bitrate=CAN_BITRATE
            )
            threading.Thread(target=self._can_rx_loop, daemon=True).start()
            self.get_logger().info("✅ CAN bus ready")
        except Exception as e:
            self.bus = None
            self.get_logger().warning(f"⚠️ CAN unavailable: {e}")
    
    def _on_scan(self, msg):
        """Process LiDAR scan and calculate control"""
        if not self.running or self.robot_stopped:
            return
        
        # Process LiDAR using LidarProcessor
        dists = LidarProcessor.process_scan(msg)
        
        # Calculate dt
        now = self.get_clock().now()
        if self.last_scan_time is None:
            dt = 0.05
        else:
            dt = np.clip((now - self.last_scan_time).nanoseconds * 1e-9, 0.01, 0.5)
        self.last_scan_time = now
        
        # Get left and right distances (ignore front wall)
        left_dist = dists['left']
        right_dist = dists['right']
        
        # Calculate steering using PID controller to center between walls
        ang = self._calculate_centering_control(left_dist, right_dist, dt)
        
        # Calculate speed based only on steering angle (no front wall)
        speed = self._calculate_speed_from_steering(abs(ang))
        
        # Clamp angular command
        ang = np.clip(ang, -ANG_LIMIT, ANG_LIMIT)
        
        # Convert to servo angle
        servo = np.clip(
            SERVO_CENTER + (STEER_GAIN * ang),
            SERVO_MIN,
            SERVO_MAX
        )
        
        # Update state
        with self.state_lock:
            self.last_servo = servo
            self.last_speed = speed
        
        # Log status periodically
        if int(time.time() * 2) % 4 == 0:
            self.get_logger().info(
                f"📏 L:{left_dist:.2f}m | R:{right_dist:.2f}m | "
                f"ang:{ang:.2f} | speed:{speed}"
            )
    
    def _imu_callback(self, msg):
        """Process IMU data and track lap completion using total rotation"""
        if self.robot_stopped:
            return
        
        # Extract quaternion from IMU message and convert to yaw
        q = msg.orientation
        raw_yaw = quaternion_to_yaw_degrees(q.x, q.y, q.z, q.w)
        
        # Initialize yaw offset on first reading
        if self.yaw_offset is None:
            self.yaw_offset = raw_yaw
            self.last_yaw = raw_yaw
            self.get_logger().info(f"🧭 Yaw offset initialized to: {self.yaw_offset:.2f}°")
            return  # Skip processing first reading
        
        # Unwrap the yaw angle to track total rotation
        unwrapped_yaw, delta_yaw = unwrap_angle_deg(raw_yaw, self.last_yaw)
        self.last_yaw = raw_yaw
        
        # Accumulate total rotation (relative to start)
        self.total_rotation += delta_yaw
        
        # Compute relative yaw for sector detection (0° = starting orientation)
        yaw = (unwrapped_yaw - self.yaw_offset) % 360
        if yaw < 0:
            yaw += 360
        
        # Log yaw periodically (every 20th callback, ~2Hz if IMU is 40Hz)
        self._imu_log_counter += 1
        if self._imu_log_counter % 20 == 0:
            self.get_logger().info(f"🧭 Yaw: {yaw:.2f}° | Total Rot: {self.total_rotation:.1f}° | Lap: {self.rotation_count}")
        
        # --- First-lap 0°→90° boost to 1300 ---
        if self.rotation_count == 0:
            # Enter boost window (only once on first lap)
            if (0.0 <= yaw <= 90.0) and not self.first90_done and not self.first90_active:
                sign = 1 if self.base_max_speed > 0 else -1
                self.params['max_speed'] = sign * 1300
                self.first90_active = True
                self.get_logger().info("🚀 First lap 0–90° → speed boost to 1300.")
        
        # Sector markers (track passages) - work for both directions
        if 80 <= yaw <= 100:
            if not self.passed_90:
                self.passed_90 = True
                self.get_logger().info("✅ Passed 90° sector")
            # Leave boost window once we've passed 90°, restore baseline
            if self.first90_active and not self.first90_done:
                self.params['max_speed'] = self.base_max_speed
                self.first90_active = False
                self.first90_done = True
                self.get_logger().info("↩️ Passed 90° on first lap → restoring baseline speed.")
        elif 170 <= yaw <= 190:
            if not self.passed_180:
                self.passed_180 = True
                self.get_logger().info("✅ Passed 180° sector")
        elif 220 <= yaw <= 300:
            if not self.passed_270:
                self.passed_270 = True
                self.get_logger().info("✅ Passed 270° sector")
                
                # Third-lap slowdown at 270°
                if self.rotation_count == 2 and not self.slowdown_done:
                    sign = 1 if self.params['max_speed'] > 0 else -1
                    self.params['max_speed'] = sign * 500
                    self.slowdown_done = True
                    self.get_logger().info("🕘 Third lap @270° → slowing to 500.")
        
        # Check if all sectors have been passed
        if not self.all_sectors_passed and self.passed_90 and self.passed_180 and self.passed_270:
            self.all_sectors_passed = True
            self.get_logger().info("🎯 All sectors passed - waiting for lap completion...")
        
        # Detect lap completion by checking if we've rotated 360° after all sectors passed
        if self.all_sectors_passed:
            rotation_threshold = 360.0 if self.total_rotation > 0 else -360.0
            if abs(self.total_rotation) >= abs(rotation_threshold):
                self.rotation_count += 1
                self.get_logger().info(f"🔁 Completed {self.rotation_count} full rotation(s). Total rotation: {self.total_rotation:.1f}°")
                
                # Reset for next lap
                self.passed_90 = False
                self.passed_180 = False
                self.passed_270 = False
                self.all_sectors_passed = False
                self.total_rotation = 0.0  # Reset rotation counter
                
                if self.rotation_count >= 3:
                    self.get_logger().info("✅ Completed 3 full rotations — move forward before stopping.")
                    self._move_forward_for_duration()
                    self.robot_stopped = True
    
    def _calculate_centering_control(self, left_dist, right_dist, dt):
        """
        Calculate steering to keep robot centered between walls with safety features
        
        Args:
            left_dist: Distance to left wall (meters)
            right_dist: Distance to right wall (meters)
            dt: Time delta (seconds)
            
        Returns:
            Angular command (radians)
        """
        # Check for emergency wall proximity
        left_valid = not np.isnan(left_dist)
        right_valid = not np.isnan(right_dist)
        
        # EMERGENCY AVOIDANCE - Override PID if too close to wall
        if left_valid and left_dist < PANIC_DISTANCE:
            self.get_logger().warn(f"🚨 PANIC LEFT! {left_dist:.2f}m - Hard right")
            self.integral = 0.0  # Reset integral
            return +PANIC_STEER_STRENGTH  # Hard right
        
        if right_valid and right_dist < PANIC_DISTANCE:
            self.get_logger().warn(f"🚨 PANIC RIGHT! {right_dist:.2f}m - Hard left")
            self.integral = 0.0  # Reset integral
            return -PANIC_STEER_STRENGTH  # Hard left
        
        # Handle missing wall readings
        if not left_valid and not right_valid:
            # No walls detected - go straight and reset
            self.integral = 0.0
            self.prev_error = 0.0
            self.smoothed_error = 0.0
            return 0.0
        
        elif not left_valid:
            # Only right wall - maintain safe distance from right
            safe_dist = self.params['min_wall_distance'] + self.params['safety_margin']
            error = right_dist - safe_dist
            # Positive error = far from right wall → turn right (negative)
            # Negative error = too close to right → turn left (positive)
            
        elif not right_valid:
            # Only left wall - maintain safe distance from left
            safe_dist = self.params['min_wall_distance'] + self.params['safety_margin']
            error = safe_dist - left_dist
            # Positive error = far from left wall → turn left (positive)
            # Negative error = too close to left → turn right (negative)
            
        else:
            # Both walls visible - CENTER between them
            # Calculate the distance difference
            # Positive error = closer to left (or right is farther) → need to turn left
            # Negative error = closer to right (or left is farther) → need to turn right
            error = right_dist - left_dist
            
            # Add safety margin bias if getting too close to either wall
            if left_dist < self.params['min_wall_distance']:
                # Too close to left - add bias to turn right
                bias = (self.params['min_wall_distance'] - left_dist) * 2.0
                error -= bias
                self.get_logger().info(f"⚠️ Close to left wall: {left_dist:.2f}m, biasing right")
            
            if right_dist < self.params['min_wall_distance']:
                # Too close to right - add bias to turn left
                bias = (self.params['min_wall_distance'] - right_dist) * 2.0
                error += bias
                self.get_logger().info(f"⚠️ Close to right wall: {right_dist:.2f}m, biasing left")
        
        # Apply exponential smoothing to reduce jitter
        self.smoothed_error = (ERROR_SMOOTHING_ALPHA * error + 
                               (1 - ERROR_SMOOTHING_ALPHA) * self.smoothed_error)
        
        # Dead zone - ignore very small errors to prevent unnecessary corrections
        if abs(self.smoothed_error) < MIN_CORRECTION_THRESHOLD:
            self.smoothed_error = 0.0
        
        # PID controller with smoothed error
        self.integral += self.smoothed_error * dt
        self.integral = np.clip(self.integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT)  # Anti-windup
        
        derivative = (self.smoothed_error - self.prev_error) / dt if dt > 0 else 0.0
        self.prev_error = self.smoothed_error
        
        # Calculate control output
        ang = (self.params['kp'] * self.smoothed_error + 
               self.params['ki'] * self.integral + 
               self.params['kd'] * derivative)
        
        return ang
    
    def _calculate_speed_from_steering(self, steering_angle):
        """
        Calculate speed based only on steering angle (no front wall dependency)
        
        Args:
            steering_angle: Absolute steering angle (radians)
            
        Returns:
            Speed command (0-2000 range)
        """
        # Start with max speed
        base_speed = self.params['max_speed']
        
        # Reduce speed when steering hard to improve stability
        if steering_angle > STEERING_THRESHOLD:
            # Calculate speed reduction factor
            reduction = ((steering_angle - STEERING_THRESHOLD) / 
                        (ANG_LIMIT - STEERING_THRESHOLD))
            reduction = np.clip(reduction, 0.0, 1.0)
            
            # Reduce speed
            target_speed = self.params['slow_speed']
            speed = int(base_speed - (base_speed - target_speed) * reduction * SPEED_REDUCTION_FACTOR)
            return max(speed, self.params['min_speed'])
        
        return base_speed
    
    def _send_can(self):
        """Send CAN commands"""
        if not self.running or self.robot_stopped or self.bus is None:
            return
        
        with self.state_lock:
            speed = int(self.last_speed)
            servo = int(self.last_servo * 100)
        
        try:
            with self.can_lock:
                self.bus.send(can.Message(
                    arbitration_id=CAN_TX_ID,
                    data=struct.pack(">ii", speed, servo),
                    is_extended_id=False
                ))
        except Exception as e:
            self.get_logger().warning(f"CAN send error: {e}")
    
    def _can_rx_loop(self):
        """CAN receive loop"""
        while rclpy.ok() and not self.rx_stop.is_set():
            try:
                if self.bus:
                    msg = self.bus.recv(timeout=1.0)
                    if msg and msg.arbitration_id == CAN_RX_ID:
                        enc1, enc2 = struct.unpack(">ii", msg.data[:8])
                        self.get_logger().debug(f"Encoders: {enc1}, {enc2}")
            except:
                pass
    
    def _check_watchdog(self):
        """Watchdog timer"""
        if self.last_scan_time and self.running:
            dt = (self.get_clock().now() - self.last_scan_time).nanoseconds * 1e-9
            if dt > SCAN_TIMEOUT:
                self.get_logger().warning(f"⏰ No scan for {dt:.1f}s - stopping")
                self._stop_robot()
    
    def _move_forward_for_duration(self, speed=0, duration_sec=0.01):
        """Move forward briefly before final stop"""
        self.get_logger().info(f"⏩ Moving forward for {duration_sec} sec before final stop...")
        
        with self.state_lock:
            self.last_speed = speed
            self.last_servo = SERVO_CENTER
        
        servo_raw = int(SERVO_CENTER * 100)
        data = struct.pack(">ii", speed, servo_raw)
        
        if self.bus:
            try:
                with self.can_lock:
                    self.bus.send(can.Message(
                        arbitration_id=CAN_TX_ID,
                        data=data,
                        is_extended_id=False
                    ))
            except Exception as e:
                self.get_logger().warning(f"CAN send error: {e}")
        
        # Allow node to process events while waiting
        self._sleep_for(duration_sec)
        
        # Final stop
        self._stop_robot()
    
    def _sleep_for(self, seconds):
        """Sleep while allowing ROS to process events"""
        end_time = self.get_clock().now().nanoseconds + int(seconds * 1e9)
        while self.get_clock().now().nanoseconds < end_time:
            rclpy.spin_once(self, timeout_sec=0.01)
    
    def _stop_robot(self):
        """Stop the robot"""
        self.running = False
        self.robot_stopped = True
        with self.state_lock:
            self.last_speed = 0
            self.last_servo = SERVO_CENTER
        
        if self.bus:
            try:
                with self.can_lock:
                    self.bus.send(can.Message(
                        arbitration_id=CAN_TX_ID,
                        data=struct.pack(">ii", 0, int(SERVO_CENTER * 100)),
                        is_extended_id=False
                    ))
            except:
                pass
        
        self.get_logger().info("🛑 Robot stopped")
    
    def _reset_lap_state(self):
        """Reset lap counting state for a fresh start"""
        self.rotation_count = 0
        self.passed_90 = False
        self.passed_180 = False
        self.passed_270 = False
        self.all_sectors_passed = False
        self.last_yaw = None
        self.yaw_offset = None  # Reset yaw offset for re-initialization
        self.total_rotation = 0.0  # Reset total rotation
        self.slowdown_done = False
        self.first90_active = False
        self.first90_done = False
        self.robot_stopped = False
        self.running = True
        self.params['max_speed'] = self.base_max_speed
        
        # Reset PID state
        self.integral = 0.0
        self.prev_error = 0.0
        self.smoothed_error = 0.0
        
        with self.state_lock:
            self.last_servo = SERVO_CENTER
            self.last_speed = 0
        
        self.get_logger().info("🔄 Lap counting state reset - ready for new run.")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = WallFollowController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node._stop_robot()
            if hasattr(node, 'rx_stop'):
                node.rx_stop.set()
            node.destroy_node()
        rclpy.shutdown()
        print("✅ Shutdown complete")


if __name__ == '__main__':
    main()

