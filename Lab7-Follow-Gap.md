# Lab 7: Follow the Gap - Reactive Obstacle Avoidance

## Lab Overview
In this lab, you will implement the "Follow the Gap" algorithm, a reactive method for obstacle avoidance. Unlike the PID-based wall following from Lab 6, this algorithm makes instantaneous decisions based on LiDAR data to find and navigate through the largest gap (free space) in front of the car. This is crucial for dynamic obstacle avoidance in racing scenarios.

**Platform:** Ubuntu 22.04 (VMware), ROS2 Humble  
**Simulator:** F1TENTH Simulator

---

## Learning Objectives
By the end of this lab, you will be able to:
1. Implement reactive obstacle avoidance algorithms
2. Process and filter LiDAR data in real-time
3. Identify free space and obstacles using range data
4. Find the largest navigable gap in sensor data
5. Select optimal goal points for navigation
6. Combine perception and control for autonomous navigation

---

## Prerequisites

Ensure ROS2 Humble and F1TENTH simulator are installed:

```bash
source /opt/ros/humble/setup.bash
ros2 pkg list | grep f1tenth
```

---
