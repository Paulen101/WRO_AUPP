Engineering materials
====

This repository contains engineering materials of a self-driven vehicle's model participating in the WRO Future Engineers competition in the season 2025.

## Content

* `t-photos` contains 2 photos of the team (an official one and one funny photo with all team members)
* `v-photos` contains 6 photos of the vehicle (from every side, from top and bottom)
* `video` contains the video.md file with the link to a video where driving demonstration exists
* `schemes` contains one or several schematic diagrams in form of JPEG, PNG or PDF of the electromechanical components illustrating all the elements (electronic components and motors) used in the vehicle and how they connect to each other.
* `src` contains code of control software for all components which were programmed to participate in the competition
* `models` is for the files for models used by 3D printers, laser cutting machines and CNC machines to produce the vehicle elements. If there is nothing to add to this location, the directory can be removed.
* `other` is for other files which can be used to understand how to prepare the vehicle for the competition. It may include documentation how to connect to a SBC/SBM and upload files there, datasets, hardware specifications, communication protocols descriptions etc. If there is nothing to add to this location, the directory can be removed.

# Introduction

This repo holds everyting we built to make our car work - from the code, to how it connects with hardware, to how you can run it yourself.

We built everything around **ROS2 Humble**, running on a **Jetson Orin Nano**. The car's fully autonomous and uses real sensors to drive, steer, and park on its own.

### What's inside

- 
-
-
-
-

### How it all connects

- 
- 
- 
- 

### How to run it 

```bash
 # Set up your ROS2 Workspace
 source /opt/ros/humble/setup.bash
 colcon build --symlink-install 
 source install/setup.bash

 # Launch the system
 ros2 launch 
```

# Our Journey

We kicked off our WRO 2025 journye with one simple but ambitious goal - build a car that can see, make decisions, and drive all by itself. No remote, no shortcuts. Just pure autonomy.

This time, we came as builders - hands-on, sleeves rolled up, ready to solve real problems.

Here's how the journey unfolded: 

We kicked off our journey experimenting with different platforms-starting with a simple Raspbot using Omni wheels and running Python scripts, then moving to TurtleBot3 powered by ROS2 Humble. Those early setups gave us just enough hands-on chaos to understand what we were really getting into. Once we got the basics down, we transitioned to the real deal: building everything from the ground up to using a Jetson Orin Nano.

None of it was easy. We had to learn ROS2 and Linux completely from scratch, troubleshoot countless bugs in our camera setup and motor logic, and rebuild our chassis more times than we can count. But every challenge leveled us up. We fine tuned our PID Controller for tight lap-following, fused sensor data for smoother control, and trained our vision system to reliably detect red and green pillars using depth data. After all htat, we finally built something we're proud of-a car that can drive, think, and even park itself without any human input.
