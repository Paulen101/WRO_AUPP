# Control Software

This directory contains all the autonomous driving algorithms and control software developed by our team for the WRO Future Engineers 2025 competition.

## 📁 Directory Structure

```
src/
├── open_challenge/          # Code for the open challenge (no obstacles)
│   ├── wall_left_following_stop.py   # Counter-clockwise wall following
│   └── wall_right_following_stop.py  # Clockwise wall following
├── obstacle_challenge/      # Code for the obstacle challenge
│   └── obstacle_right.py    # Obstacle avoidance with pillar detection
├── best.pt                 # YOLOv8 trained model for pillar detection
└── README.md              # This file
```

## 🤖 Our Approach: Camera-Only Navigation

Since our car only has a camera as the primary sensor - specifically the Intel RealSense D455 - we developed a unique approach that combines depth perception and color vision for autonomous navigation. This camera-centric design is what drives our wall-following strategy and obstacle detection algorithms throughout the entire system.

The reason we chose wall following as our core navigation method comes down to practical limitations and advantages of camera-based sensing. With only a camera to work with, wall following gives us the most reliable way to navigate because walls provide consistent reference points that our depth camera can measure accurately. This approach is also much more robust when lighting conditions change compared to pure visual navigation, and it creates smooth, repeatable paths around the track while maintaining safe distances from track boundaries.

## 🏁 Open Challenge Algorithms

The open challenge requires our car to complete three laps around the track without any obstacles in the way. We implemented two complementary approaches that work together to handle different track configurations and directions.

Our wall following algorithm works on a simple but effective principle: use the depth camera to maintain a target distance from the wall while constantly scanning ahead for obstacles that would require turning. The core of this system is a PID controller that calculates the error between our target distance (0.65 meters) and the actual measured distance to the wall, then adjusts the steering accordingly. We extract specific regions of interest from the depth image to get clean distance measurements, and when the front obstacle detection triggers (usually when approaching a wall), the car knows it's time to make a turn.

We built two different implementations to handle both directions of travel. The left wall following approach is designed for counter-clockwise navigation, where the steering control adds the calculated offset to the base 90-degree position, and front obstacles trigger a left turn by setting the servo to 45 degrees. The right wall following version handles clockwise navigation by inverting the offset calculation and triggering right turns to 135 degrees when obstacles appear ahead.

Our lap counting system uses the IMU's yaw data to track progress through specific angle thresholds at 90, 180, and 270 degrees. When the car completes a full rotation back to the starting position, we increment the lap counter. The system also includes dynamic speed management that gives a boost during the first part of the first lap, then slows down during the third lap before automatically stopping after three complete rotations.

## 🚧 Obstacle Challenge Algorithm

The obstacle challenge adds red and green pillars that our car needs to detect and avoid while still completing the course. This is where things get much more complex because we need to combine our wall following with AI-powered pillar detection.

For pillar detection, we use a custom trained model called `best.pt` that's based on both YOLOv8n and YOLOv8s architectures. We collected over 5000 images from various sources including internet datasets and our own photography sessions under different lighting conditions. The training process resulted in a model that's both accurate enough for reliable detection and fast enough to run in real-time on our Jetson Orin Nano. Honestly, the model itself turned out great and wasn't really the problem - most of our headaches came during the implementation phase when we were trying to write algorithms that could effectively use the model's predictions to control the car.

The biggest challenge we faced wasn't getting the model to detect pillars correctly, but rather figuring out how to translate those detections into smart driving decisions. We ended up developing a zone-based avoidance strategy where we divide the camera's field of view into left, middle, and right zones based on where the pillar appears. Green pillars in the middle zone trigger a moderate left turn, but if they're in the right zone we ignore them since they're too far away to matter. Red pillars work the opposite way - middle zone triggers a right turn, but left zone pillars are ignored.

We also built a multi-layer safety system to handle situations where pillar detection fails or gives unexpected results. When no pillar is detected but our front sensors show an obstacle in the danger zone (between 0.30 and 0.40 meters), the car automatically executes a two-stage avoidance maneuver. First, it does a pre-turn away from the wall for about 1.3 seconds, then reverses while steering for 2 seconds before returning to normal operation. If all else fails, the system falls back to standard wall following using just the depth data.

The whole obstacle challenge runs on a state machine architecture that handles different behaviors like normal driving, pillar avoidance, emergency backoff maneuvers, wall following, and stopping. Each state manages specific behaviors and transitions, which helps ensure the car keeps working even when sensors give unexpected data or when our algorithms encounter edge cases we didn't think of during development.

## 🔧 Technical Implementation Details

The hardware communication happens through a CAN bus for motor and servo control, with ROS2 handling all the inter-process communication and sensor data. The Intel RealSense provides both depth and color streams at 640x480 resolution running at 30 frames per second, which gives us enough detail for both wall following and pillar detection.

Our computer vision pipeline starts by grabbing aligned depth and color frames, then extracts regions of interest for distance measurements. We run YOLO inference on the color frames for real-time pillar detection, then combine the depth-based wall following with color-based obstacle avoidance to make final decisions. The whole process from sensor input to control output typically takes less than 100 milliseconds, which gives us real-time responsiveness.

We optimized everything for performance by focusing processing on specific regions of interest rather than analyzing entire frames, which reduces computational load significantly. The system also includes robust error recovery mechanisms to handle sensor failures, CAN bus errors, and other edge cases that could otherwise stop the car dead in its tracks.

This camera-only approach shows how computer vision can replace traditional sensor arrays like LIDAR or ultrasonic sensors, making autonomous vehicles more accessible while still maintaining high performance and safety standards. The biggest lesson we learned is that having a good AI model is only half the battle - the real challenge is writing algorithms that can effectively use that model's output to make smart driving decisions in real-world conditions.