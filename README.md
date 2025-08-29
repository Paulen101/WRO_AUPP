# WRO Future Engineers 2025 - Our Autonomous Vehicle
*Everything we built to make it work*

---

## 🎯 What We Built

This repository holds all the engineering work behind our autonomous vehicle for the **WRO Future Engineers competition 2025**. We've built a car that can see, think, and drive completely on its own - no remote control, no human input, just pure autonomy.

**What makes it special:**
- 🚗 Drives itself from start to finish without any help
- 🎯 Spots and avoids obstacles in real-time
- 📷 Uses computer vision to recognize red and green pillars
- 🧠 Makes smart decisions while racing
- 🎮 Fine-tuned control systems that keep it smooth and fast

---

## 👥 Meet Our Team

*[Team photo placeholder - group photo showing all team members and coach]*

### **Team Member 1**
*[Photo placeholder]*
- **Role**: [Role/Specialty]
- **Bio**: [Brief description of their contribution and background]

### **Team Member 2** 
*[Photo placeholder]*
- **Role**: [Role/Specialty]
- **Bio**: [Brief description of their contribution and background]

### **Team Member 3**
*[Photo placeholder]*
- **Role**: [Role/Specialty] 
- **Bio**: [Brief description of their contribution and background]

### **Coach**
*[Photo placeholder]*
- **Role**: Team Coach/Mentor
- **Bio**: [Brief description of their guidance and background]

---

## 📁 What's In Here

### The Documentation 
* **`t-photos/`** - Photos of our team (the official one and some fun shots)
* **`v-photos/`** - Pictures of our car from every angle you could want
* **`video/`** - Videos showing how our car actually drives
* **`schemes/`** - All the wiring diagrams and how everything connects

### The Technical 
* **`src/`** - The code that makes everything work
* **`models/`** - 3D files for parts we designed and printed ourselves
* **`other/`** - Extra documentation and setup guides

---

## 🚀 How We Built It

We built our car around some pretty solid hardware and wrote custom software to make it all work together:

### **The Hardware**
- **Brain**: NVIDIA Jetson Orin Nano (this thing is powerful!)
- **OS**: Ubuntu 22.04 with ROS2 Humble running the show
- **Eyes**: RGB-D camera so it can see in 3D and recognize colors
- **Sensors**: Ultrasonic sensors, IMU, and wheel encoders to know what's happening around it
- **Movement**: Servo motors for steering and driving with precision

### **The Software**
- **Framework**: ROS2 Humble - it's like the nervous system connecting everything
- **Vision**: OpenCV with our own algorithms to spot those red and green pillars
- **Control**: PID controllers that we spent way too much time tuning (but it was worth it!)
- **Decision Making**: Smart algorithms that combine data from all sensors

---

## 🔧 What Our Car Can Do

### **Driving Itself**
- Follows walls to navigate the track smoothly
- Uses an AI model to keep track of obstacles and avoid them
- Uses sensors and algorithms to stay in bounds
- Adjusts speed based on what's ahead

### **Seeing and Understanding**
- **Pillar Detection**: Spots red and green pillars even when lighting changes
- **Depth Vision**: Knows exactly how far away things are
- **AI-Powered Tracking**: Uses machine learning to monitor and predict obstacle movements

### **Smart Control**
- **Steering**: PID controllers we fine-tuned until they were perfect
- **Speed**: Knows when to go fast and when to slow down
- **Safety**: Emergency stops when something unexpected happens

---

## 🏁 How It Performs

Right now, our car can handle:

1. **Open Challenge**: Completing laps autonomously with smooth, optimized racing lines
2. **Obstacle Challenge**: Avoiding obstacles while maintaining good speed
   - *Note: We're still working on the parking part - it's integrated but not quite there yet*

### **The Numbers**
- **Speed**: Fast enough to be competitive, smart enough to stay safe
- **Vision Range**: Can see obstacles and pillars from a good distance
- **Response Time**: Reacts in real-time (under 100ms)
- **Accuracy**: Rarely misses a pillar, even in tricky lighting

---

## 🛠️ Our Journey

### **Starting Small**
We began with simpler setups - a Raspbot with omniwheels and a TurtleBot3 - just to get our feet wet with robotics and understand how ROS2 works.

### **Going Custom**
Once we felt confident, we decided to build everything from scratch using the Jetson Orin Nano. This is where things got real.

### **Learning the Hard Way**
- Had to learn ROS2 and Linux from zero (lots of late nights!)
- Spent countless hours debugging camera issues and motor control
- Rebuilt our chassis more times than we care to admit
- But every problem taught us something new

### **Making It Work**
- Fine-tuned our PID controllers until the car drove like a dream
- Got our vision system to reliably spot pillars in all kinds of lighting
- Integrated all our sensors so they work together seamlessly
- Built something we're genuinely proud of

---

## 🏆 What Makes It Special

- **100% Autonomous**: Once it starts, we don't touch it - the car handles everything
- **Smart Vision**: Recognizes pillars even when the lighting isn't perfect
- **Easy to Upgrade**: We designed it so we can easily add new features or improve existing ones

---

## 🚀 Want to Try It?

Everything you need to build and run our car is in this repository. We've documented each part thoroughly so you can understand how it works and even improve on our design.

---

*This represents months of hard work, learning, and problem-solving. We started with a simple goal - build a car that could drive itself - and ended up with something that genuinely impressed us. Hope it inspires you too!*
