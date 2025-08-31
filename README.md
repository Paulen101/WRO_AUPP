# WRO Future Engineers 2025 - TeamBaymax Motion (AUPP)

<style>
/* Mobile responsive styles */
@media screen and (max-width: 768px) {
  table, tr, td {
    display: block !important;
    width: 100% !important;
  }
  
  td[width="30%"], td[width="70%"] {
    width: 100% !important;
    text-align: center !important;
    padding: 10px !important;
  }
  
  /* Ensure name/photo always appears before bio on mobile */
  td[width="30%"] {
    order: -1 !important;
  }
  
  tr {
    display: flex !important;
    flex-direction: column !important;
  }
  
  img {
    max-width: 100% !important;
    height: auto !important;
  }
  
  h4 {
    font-size: 16px !important;
    margin: 5px 0 !important;
  }
}

@media screen and (max-width: 480px) {
  img[width="300"] {
    width: 200px !important;
  }
  
  img[width="1000"] {
    width: 100% !important;
  }
}
</style>

<div align="center">
<img src="other/banner.svg">
</div>

<div align="left" style="margin: 20px 0;">
  <a href="https://www.instagram.com/teambaymaxmotion?utm_source=qr" target="_blank">
    <img src="https://img.shields.io/badge/Instagram-E4405F?style=for-the-badge&logo=instagram&logoColor=white" alt="Instagram" />
  </a>
  &nbsp;&nbsp;
  <a href="https://www.youtube.com/channel/your_youtube_channel" target="_blank">
    <img src="https://img.shields.io/badge/YouTube-FF0000?style=for-the-badge&logo=youtube&logoColor=white" alt="YouTube" />
  </a>
</div>

---
*"Apparently we needed to teach a car how to drive and follow traffic rules since humans keep forgetting." 🤷‍♂️🏎️💨*

---

## 📚 Table of Contents

- [WRO Future Engineers 2025 - TeamBaymax Motion (AUPP)](#wro-future-engineers-2025---teambaymax-motion-aupp)
  - [📚 Table of Contents](#-table-of-contents)
  - [🎯 What We Built](#-what-we-built)
  - [👥 Meet Our Team](#-meet-our-team)
  - [📁 What's In Here](#-whats-in-here)
    - [The Documentation](#the-documentation)
    - [The Technical](#the-technical)
  - [🚀 How We Built It](#-how-we-built-it)
    - [**Choosing Our Hardware**](#choosing-our-hardware)
    - [**Writing the Software**](#writing-the-software)
    - [**Making Everything Work Together**](#making-everything-work-together)
  - [🔧 What Our Car Can Do](#-what-our-car-can-do)
    - [**Driving Itself**](#driving-itself)
    - [**Seeing and Understanding**](#seeing-and-understanding)
    - [**Smart Control**](#smart-control)
  - [🏁 How It Performs](#-how-it-performs)
    - [**The Numbers**](#the-numbers)
  - [🛠️ Our Journey](#️-our-journey)
    - [**Starting Small**](#starting-small)
    - [**Going Custom**](#going-custom)
    - [**Learning the Hard Way**](#learning-the-hard-way)
    - [**Making It Work**](#making-it-work)
  - [🏆 What Makes It Special](#-what-makes-it-special)
  - [🚀 Want to Try It?](#-want-to-try-it)
  - [📜 Copyright](#-copyright)

---

## 🎯 What We Built

This repository holds all the engineering work behind our autonomous vehicle for the **WRO Future Engineers competition 2025**. We've built a car that can see, think, and drive completely on its own - no remote control, no human input, just pure autonomy.

**What makes it special:**

Our car drives itself from start to finish without any help, spotting and avoiding obstacles in real-time using advanced computer vision to recognize red and green pillars. It makes smart decisions while navigating through challenges, all powered by fine-tuned control systems that keep everything smooth and fast.

---

## 👥 Meet Our Team

<div align="center">
<img src="t-photos/team-photo-ft-coach.jpeg" width="1000" style="max-width: 100%; height: auto;">
<p><em>Team Photo - From left to right: Paulen, Bunkheang, Panha, and Prof. Theara (Coach)</em></p>
</div>

---

<table width="100%">
<tr>
<td width="30%" valign="top" align="center">
<h4><strong>Prof. SENG Theara </strong></h4>
<img src="t-photos/profTheara.svg" width="300" style="display: block; margin: 10px auto;">
</td>
<td width="70%" valign="middle">
<h4><strong>Role:</strong> Team Coach</h4>
<h4><strong>Origin:</strong> Siem Reap, Cambodia</h4>
<h4><strong>Email:</strong> <a href="mailto:t.seng@aupp.edu.kh">t.seng@aupp.edu.kh</a></h4>
<strong>Bio:</strong> An innovative tech enthusiast and lecturer at the American University of Phnom Penh (AUPP) with a Master's degree in Robotics from France. He has experiences in microcontroller programming, sensor integration, and IoT systems. As our team coach, he provided expert guidance on robotics fundamentals, autonomous systems, and AI integration while mentoring us through complex technical challenges and project development. His passion for cutting-edge technologies and hands-on approach to teaching helped shape our understanding of advanced robotics and prepared us for the WRO competition.
</td>
</tr>
</table>

---

<table width="100%">
<tr>
<td width="70%" valign="middle">
<h4><strong>Role:</strong> 3D Design & Software Developer</h4>
<h4><strong>Origin:</strong> Battambang, Cambodia</h4>
<h4><strong>Email:</strong> <a href="mailto:2024033chamroeun@aupp.edu.kh">2024033chamroeun@aupp.edu.kh</a></h4>
<strong>Bio:</strong> A junior at the American University of Phnom Penh (AUPP) majoring in Information and Communication Technology (ICT). Specialized in 3D modeling and vehicle design using CAD software. Responsible for designing and optimizing the car's 3D components and mechanical structure. Also contributed to software development and played a key role in project planning and coordination. His academic background in ICT combined with 3D design expertise helped create the physical foundation of our autonomous vehicle.
</td>
<td width="30%" valign="top" align="center">
<h4><strong>CHAMROEUN Vireakpanha</strong></h4>
<img src="t-photos/panha.svg" width="300" style="display: block; margin: 10px auto;">
</td>
</tr>
</table>

---

<table width="100%">
<tr>
<td width="30%" valign="top" align="center">
<h4><strong>HENG BunKheang</strong></h4>
<img src="t-photos/kheang.svg" width="300" style="display: block; margin: 10px auto;">
</td>
<td width="70%" valign="middle">
<h4><strong>Role:</strong> Lead Software Developer</h4>
<h4><strong>Origin:</strong> Phnom Penh, Cambodia</h4>
<h4><strong>Email:</strong> <a href="mailto:2023302heng@aupp.edu.kh">2023302heng@aupp.edu.kh</a></h4>
<strong>Bio:</strong> A junior at the American University of Phnom Penh (AUPP) and Fort Hays State University (FHSU), majoring in Information Technology Management/Computer Science (ITM/CS) with a focus on software engineering and robotics. Serves as the lead programmer and system architect for the autonomous vehicle project. Specialized in ROS2 development, sensor integration, and control systems. Responsible for the overall software architecture, PID controller implementation, and real-time decision-making algorithms. His comprehensive programming skills and system design experience ensure seamless integration between all components of our autonomous vehicle.
</td>
</tr>
</table>

---

<table width="100%">
<tr>
<td width="70%" valign="middle">
<h4><strong>Role:</strong> Hardware & Software Developer</h4>
<h4><strong>Origin:</strong> Battambang, Cambodia</h4>
<h4><strong>Email:</strong> <a href="mailto:2023225chhun@aupp.edu.kh">2023225chhun@aupp.edu.kh</a></h4>
<strong>Bio:</strong> A junior at the American University of Phnom Penh (AUPP) and Fort Hays State University (FHSU), majoring in Information Technology Management/Computer Science (ITM/CS). Specialized in mechanical and hardware work while also contributing to software development. Has been instrumental in car building from the early TurtleBot3 experiments to the final autonomous vehicle. Worked on both software and hardware integration throughout the project's evolution, providing crucial support in mechanical assembling, component integration, and system troubleshooting.
</td>
<td width="30%" valign="top" align="center">
<h4><strong>CHHUN Paulen</strong></h4>
<img src="t-photos/paulen.svg" width="300" style="display: block; margin: 10px auto;">
</td>
</tr>
</table>
</td>

---

## 📁 What's In Here

### The Documentation 
* **`t-photos/`** - Photos of our team (the official one and some fun shots)
* **`v-photos/`** - Pictures of our car from every angle you could want
* **`video/`** - Videos showing how our car actually drives
* **`schemes/`** - All the wiring diagrams and how everything connects

### The Technical 
* **`src/`** - The code that makes everything work, including trained AI models (YOLOv8 for obstacle detection)
* **`models/`** - 3D files for parts we designed and printed ourselves, including AI models that we use for computer vision
* **`other/`** - Extra documentation and setup guides

---

## 🚀 How We Built It

Building our autonomous vehicle was quite the journey - lots of trial and error, late nights debugging, and those "aha!" moments when everything finally clicked. Here's how we turned our crazy idea into reality:

### **Choosing Our Hardware**
We had to be smart about every component since we're students on a budget (aka broke), but we also needed serious performance. Also these components are also selected by our coach as well because he knows everything.

- **The Brain**: NVIDIA Jetson Orin Nano Super - This little computer is honestly incredible. It's got ARM64 architecture with AI acceleration that lets us run our machine learning models without breaking a sweat
- **Operating System**: Ubuntu 22.04.5 LTS - We chose this because it's rock-solid stable, and we needed something that wouldn't crash during competitions
- **The Eyes**: Intel RealSense D455 camera - This was a game-changer for us. It gives us both regular color vision and depth perception, so our car can actually "see" in 3D
- **Balance and Navigation**: BNO055 IMU sensor that tells us exactly how our car is tilted, turning, or moving. Think of it as our car's inner ear
- **Movement**: Custom motor setup with encoders - We needed precise control, so we went with motors that can tell us exactly where they are and how fast they're spinning
- **The Body**: We 3D-designed and printed our own chassis. After breaking a few prototypes, we finally got one that could handle all our gear and look good doing it

### **Writing the Software**
This is where things got really interesting (and mathematical, if we're being honest):

We didn't just jump into coding - our coach taught us the theory behind everything first. We spent hours going through equations, learning about different algorithms and methods that could work for autonomous navigation. Then came the fun part: experimenting with these concepts ourselves, both independently and with our coach's guidance.

- **The Foundation**: ROS2 Humble became our best friend - it's like having a universal translator that lets all our different components talk to each other seamlessly
- **Teaching It to See**: We spent weeks training YOLOv8 AI models to spot those red and green pillars. We used both YOLOv8s and YOLOv8n versions, created our own dataset by taking hundreds of photos in different lighting conditions, and then optimized everything into .engine files so it runs super fast on our Jetson
- **Making It Drive Smoothly**: PID controllers sound simple in theory, but wow, did we spend countless hours tweaking those parameters and working through the math behind them. Our coach helped us understand the equations that make these controllers work, then we experimented with different tuning methods. Every tiny adjustment changed how our car behaved, but once we got it right, it felt like magic
- **Putting It All Together**: We built algorithms that take input from our camera, IMU, and motor sensors, then make split-second decisions about where to go and how fast to get there. This involved lots of mathematical calculations and algorithmic thinking that our coach guided us through
- **Keeping It Safe**: We learned the hard way to always have backup plans. Emergency stops, error recovery, and fail-safes became our saving grace during those "oh no" moments

### **Making Everything Work Together**
The real challenge wasn't building individual parts - it was getting them to play nice together. We spent weeks calibrating sensors, testing under different lighting conditions, and fine-tuning everything until our car could handle whatever the competition threw at it. Every bug we fixed and every improvement we made brought us closer to having something we could be truly proud of.

---

## 🔧 What Our Car Can Do

### **Driving Itself**

Think of our car as having learned to drive just like a human would, but maybe even better. It naturally follows the walls to find its way around the track, almost like running your hand along a hallway in the dark. While it's doing this, it's constantly watching out for those red and green pillars, spotting them from a distance and smoothly steering around them without breaking a sweat. The car has this incredible sense of where it belongs on the track - it just knows not to wander off course. What really impresses us is how it reads the situation ahead and decides whether to speed up through open areas or slow down when things get tricky, kind of like how you'd naturally adjust your pace when walking through a crowded room versus an empty hallway.

### **Seeing and Understanding**

Our car's vision is honestly pretty amazing - it's like giving it a pair of really smart eyes that never get tired. It can spot those red and green pillars from way across the room, even when the lighting is being weird or there are shadows everywhere. But it's not just seeing them - it actually understands how far away everything is, kind of like having built-in depth perception that's probably better than ours. The coolest part is how it uses machine learning to not just see what's there right now, but to predict where things might move next, almost like it's thinking ahead about what the obstacles might do.

### **Smart Control**

The way our car controls itself feels almost human-like, but probably more precise than most of us could ever be. We spent ages fine-tuning these PID controllers until they were just right - now the steering is so smooth it's like the car just naturally knows where to go. It has this intuitive sense of when to floor it on the straightaways and when to ease up before a tricky turn, kind of like an experienced driver who's driven the same route a thousand times. And just like any good driver, it always has its finger on the brake - if something unexpected pops up, it can stop on a dime to keep everyone safe.

---

## 🏁 How It Performs

Right now, our car can handle:

1. **Open Challenge**: Completing laps autonomously with smooth, optimized racing lines
2. **Obstacle Challenge**: Avoiding obstacles while maintaining good speed
   - *Note: We're still working on the parking part - it's integrated but not quite there yet*

### **The Numbers**
When it comes to speed, our car has found that sweet spot where it's fast enough to actually compete but smart enough not to be reckless - though we'll be honest, it still occasionally kisses the wall during testing (we're working on that!). The vision system can spot obstacles and pillars from impressively far away, giving it plenty of time to plan its moves. What really blows our minds is how quick it thinks - we're talking reaction times under 100 milliseconds, which is faster than most humans can even blink. And here's something we're genuinely proud of: it almost never misses spotting a pillar, even when the lighting conditions are being absolutely terrible and would probably fool us.

---

## 🛠️ Our Journey

### **Starting Small**
We began with simpler setups - a Raspbot with omniwheels and a TurtleBot3 - just to get our feet wet with robotics and understand how ROS2 works.

### **Going Custom**
Once we felt confident, we decided to build everything from scratch using the Jetson Orin Nano. This is where things got real.

### **Learning the Hard Way**
We had to start from scratch with ROS2 and Linux, which meant plenty of late nights figuring out how everything worked. Debugging camera issues and getting the motor control just right took more time than we expected, and we definitely went through a few chassis designs before landing on one that actually worked well. But honestly, every challenge we hit taught us something valuable, and looking back, those problem-solving sessions were where we learned the most.

### **Making It Work**
Getting our PID controllers dialed in took patience, but once we found the right settings, the car's driving became really smooth and predictable. We put a lot of effort into making our vision system reliable across different lighting conditions, and it was satisfying to see it consistently spot those pillars. The real achievement was getting all our sensors to work together seamlessly - when everything finally clicked into place, we knew we had built something solid that we could be proud of.

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

---

## 📜 Copyright


**MIT License**

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

**THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.**

**© 2025 Team Baymax Motion**  
*American University of Phnom Penh (AUPP), Cambodia*
