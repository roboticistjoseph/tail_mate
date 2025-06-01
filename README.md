## TailMate: ROS2 Drone Follower & ArUco Tracker


## Table of Contents

1. [Project Description](#project-description)
2. [Features & Tracking Evolution](#features--tracking-evolution)
3. [Installation](#installation)
4. [Cloning the Repository](#cloning-the-repository)
5. [Adding Submodules](#adding-submodules)
6. [Modifying the TurtleBot3 Model](#modifying-the-turtlebot3-model)
7. [Detailed Project Breakdown](#detailed-project-breakdown)
8. [Future Works](#future-works)

---

## Project Description

- **TailMate** is a ROS2-based drone follower project that lets a drone visually track and follow a TurtleBot3 robot using computer vision. The project started with classic techniques like color detection, contour detection, and ORB feature matching, then moved to advanced template matching (which worked surprisingly well!). But for real-world robustness, ArUco marker tracking was the winner—and that’s what powers the current system.

- The node subscribes to the drone’s camera feed, detects ArUco markers, calculates PID and publishes velocity commands to follow a specific marker (ID 3). The code is modular, well-documented, and ready for both simulation and real hardware.

---

## Features & Tracking Evolution

| Method                | Result/Notes                                                                 |
|-----------------------|------------------------------------------------------------------------------|
| Color Detection       | Simple, but unreliable under changing lighting                               |
| Contour Detection     | Good, but sensitive to noise                        |
| ORB Feature Matching  | High flcutuation of centroid depending on detected features                  |
| Template Matching     | Upgraded version gave great results, but not robust to all conditions        |
| **ArUco Tracking**    | **Rock-solid, robust, and reliable—now the default for this project!**       |

---

## Installation

**Prerequisites:**
- ROS2 (tested on Humble)
- OpenCV (with ArUco module)
- C++ build tools

**Install required ROS2 and OpenCV packages:**
```bash
sudo apt update
sudo apt install ros-humble-desktop python3-opencv libopencv-dev
```

---

## Cloning the Repository

```bash
git clone https://github.com/roboticistjoseph/tail_mate.git
cd tail_mate
```

---

## Adding Submodules

This project uses two submodules for simulation and drone models:

```bash
# Add the SJTU drone submodule (original repo)
git submodule add -b ros2 https://github.com/NovoG93/sjtu_drone.git

# Add the TurtleBot3 simulations submodule
git submodule add -b humble https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

# Initialize and update submodules
git submodule update --init --recursive
```

---

## Modifying the TurtleBot3 Model

To enable visual tracking, you’ll need to:
- **Change the TurtleBot3’s texture**
- **Add an ArUco marker to the model’s SDF file**

**Instructions:**  
See the detailed README in the `assets` folder for step-by-step guidance on editing the TurtleBot3 model.

---

## Detailed Project Breakdown

For a deep technical dive—including code walkthroughs, design decisions, and demo videos—visit:  
👉 [Detailed Breakdown](https://josephkatakam.vercel.app/projects/ros2_follower_drone)

---

## Future Works

- **Swarm Mode:** Spawn and control multiple drones for coordinated following and coverage.
- **Surveillance:** Extend the system for area monitoring, patrol, and smart event detection.
---