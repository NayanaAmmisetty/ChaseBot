# 🛩️ ChaseBot Enhanced Simulation – Aircraft Target Tracking System

### 🔍 Overview
This project simulates a real-time multi-agent system where autonomous bots (representing aircraft or missiles) detect, track, and intercept moving aerial targets based on color. It integrates **computer vision**, **motion planning**, **obstacle avoidance**, and **sound feedback**—emulating real-world **aerial surveillance and targeting scenarios**.

---

### ✨ Key Features
- 🎯 **Target Detection & Tracking**: Bots identify and pursue colored aircraft (green, pink, blue) using live video feed and HSV-based color segmentation.
- 🧠 **Dynamic Obstacle Avoidance**: 8 circular, independently moving obstacles challenge the bots, which navigate using a custom real-time avoidance algorithm.
- 🔊 **Intelligent Sound Feedback**: 
  - **Chime** when a bot reaches its target.
  - **Beep** when a bot gets dangerously close to an obstacle.
- 📈 **3D Visualization**:
  - Real-time motion paths of bots and targets.
  - Interactive 3D plots showing X-Y-time trajectories.
- 🌀 **Trigger Zones & Target Contact Visualization**:
  - Visual triggers appear when bots are near targets.
  - Large colored markers indicate successful interceptions.

---

### ✈️ Real-World Inspiration: Aircraft Surveillance & Interception
This simulation draws direct parallels to **aerial defense systems**, such as:
- **Unmanned Aerial Vehicles (UAVs)** tracking enemy aircraft.
- **Missile Guidance Systems** adjusting flight paths in real time to avoid defense countermeasures (obstacles) while homing in on targets.
- **Autonomous Target Tracking Drones** in defense, border patrol, or search-and-rescue operations.

By modeling these behaviors, this prototype offers insight into how **vision-based autonomous agents** can make strategic, environment-aware decisions in dynamic, multi-target settings.

---

### 🧰 Tech Stack & Components
| Component                | Description                                     |
|-------------------------|-------------------------------------------------|
| **MATLAB**              | Simulation environment                          |
| **Webcam**              | Real-time video capture                         |
| **Computer Vision**     | HSV-based color tracking                        |
| **Path Planning**       | Directional movement + obstacle avoidance       |
| **Obstacle Simulation** | 8 circular moving obstacles with velocity logic |
| **Audio Feedback**      | `chime.wav`, `beep.wav` for bot state awareness |
| **Visualization**       | `imshow`, `plot3`, `viscircles`, `rectangle`    |

---

### 🧪 Simulation Demo Flow
1. **Initialize camera and environment**
2. **Detect colored moving objects**
3. **Track positions frame-by-frame**
4. **Move each bot toward its respective target**
5. **Avoid collisions with moving obstacles**
6. **Visual + audio indicators show detection and proximity events**
7. **Simulation ends when all bots reach their targets**

---

### 📊 Sample Outputs
- 🧭 **Live Bot Movement**: Displays real-time motion of bots and targets with color-coded paths.
- 🌐 **3D Graphs**: Post-simulation plots showing movement over time in 3D space.
- ⚠️ **Warnings and Alerts**: Visual trigger rings + audio cues enhance situational awareness.

---

### 🚀 Future Scope
- Integrate **Kalman Filters** or **Optical Flow** for smoother tracking in noisy environments.
- Add **3D drone movement logic** and **altitude estimation** for realistic aerial dynamics.
- Port to **ROS2 and Gazebo** for robotics hardware integration or UAV simulation.

---

### 📂 Repository Contents
- `ChaseBot_Enhanced.m` – Main simulation code
- `chime.wav` & `beep.wav` – Sound effects
- `README.md` – Project description and application
- Helper functions:
  - `getLargestRegion()` – Extracts the largest detected object
  - `drawBot()` – Draws circular aircraft representations
  - `improvedAvoidance()` – Avoids obstacles during pursuit

---

### 🤖 Author
**Nayana Ammisetty** – 2nd Year BTech | AI Enthusiast | Robotics Innovator  
Passionate about building intelligent, real-world simulations for aerial and ground robotic systems.

---

### 📌 Citation
If you find this work useful for research or educational purposes, feel free to cite or fork the repo. Contributions welcome!

