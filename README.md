# 🤖 Autonomous Food Delivery Robot System
### ROS-Based Restaurant Service Robot | RAD 334 Mini Project

**Student:** Sayanth S Kumar | **Reg No:** ASI24RA046  
**Department:** Robotics & Automation | **Institution:** ASIET Kalady  
**University:** APJ Abdul Kalam Technological University (KTU)  
**Guide:** Mr. Arun Kumar

---

## 📌 Project Overview

An autonomous food delivery robot that navigates a simulated restaurant environment using ROS Noetic. The robot picks up food from the kitchen and delivers it to any of 6 dining tables using an 8-stage Finite State Machine (FSM), dual-highway corridor routing, and real-time LiDAR obstacle safety interlock.

---

## ✨ Key Features

- 🏗️ **20×15 m Gazebo Cafeteria Simulation** with 6 dining tables, kitchen, and front desk
- 🤖 **Differential Drive Robot** modeled in URDF/Xacro with motorized food compartment lid
- 🧠 **8-Stage FSM Action Server** (IDLE → Kitchen → Loading → Navigating → At Table → Returning)
- 🛣️ **Dual-Highway Corridor Routing** (Central Aisle + East Aisle) for conflict-free delivery
- 🚨 **LiDAR Obstacle Auto-Stop** (±20° frontal cone, stops at 0.38m, resumes at 0.50m)
- 📡 **Custom ROS Messages/Services/Actions** (`NavigateToGoal.action`, `RobotStatus.msg`)
- 🖥️ **Interactive Operator CLI** to dispatch delivery goals to any table

---

## 📦 Package Structure

```
catkin_ws/src/
├── food_robot_msgs/          # Custom messages, services, and actions
├── food_robot_description/   # URDF/Xacro robot model
├── food_robot_gazebo/        # Cafeteria world + launch files
├── food_robot_control/       # FSM Action Server + Status Publisher
├── food_robot_mapping/       # GMapping SLAM config + pre-built map
├── food_robot_navigation/    # Nav Stack (move_base, AMCL) config
└── food_robot_bringup/       # Full system launcher + CLI dispatcher
```

---

## 🗺️ Delivery Coordinates

| Destination | X (m) | Y (m) | Corridor |
|-------------|--------|--------|----------|
| Kitchen (Home) | 7.5 | 0.0 | Home Base |
| Table 1 | 1.5 | 2.0 | Central Aisle |
| Table 2 | 1.5 | 4.5 | Central Aisle |
| Table 3 | 1.5 | 6.2 | Central Aisle |
| Table 4 | 5.5 | 2.0 | East Aisle |
| Table 5 | 5.5 | 4.5 | East Aisle |
| Table 6 | 5.5 | 6.2 | East Aisle |
| Front Desk | 2.0 | -1.5 | Direct Path |

---

## 🚀 How to Run

### Prerequisites
- Ubuntu 20.04 (or WSL2)
- ROS Noetic
- Gazebo 11

### Build & Launch

```bash
# Clone the repository
git clone https://github.com/YOUR_USERNAME/food-delivery-robot.git
cd food-delivery-robot

# Build the workspace
cd catkin_ws
catkin_make
source devel/setup.bash

# Terminal 1 - Start ROS Master
roscore

# Terminal 2 - Launch Cafeteria World
roslaunch food_robot_gazebo cafeteria.launch

# Terminal 3 - Start Action Server
rosrun food_robot_control navigate_action_server_v2.py

# Terminal 4 - Operator CLI
rosrun food_robot_bringup send_goal_example.py
```

---

## 🔄 FSM Delivery Lifecycle

```
IDLE → GOTO_KITCHEN → LOADING (30s) → NAVIGATING → AT_TABLE (120s) → RETURNING → IDLE
                                           ↕
                                    OBSTACLE_STOP ↔ (auto-resume)
                                           ↕
                                       PREEMPTED (on cancel)
```

---

## 📊 Expected Outcomes

| Metric | Value |
|--------|-------|
| Delivery Success Rate | ~95% |
| Obstacle Response Latency | ~87 ms |
| Full Delivery Cycle | 160–400 seconds |
| Stop Distance | 0.38 m |
| Resume Distance | 0.50 m |

---

## 📜 License
MIT License — Free to use for academic purposes.
