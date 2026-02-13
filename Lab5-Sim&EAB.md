# Lab5: F1TENTH Gym ROS Setup Guide for ROS2 Humble on Ubuntu 22.04

This guide will walk you through the steps to install the **F1TENTH Gym** environment (a Python-based simulation library) and integrate the ROS2 communication bridge. These instructions apply to both **ROS 2 Humble** running on **Ubuntu 22.04** (either natively or through a virtual machine).

---

## 1. Update System and Install Dependencies

Before starting, ensure your system packages are up-to-date and that Python3’s `pip` is installed:

```bash
sudo apt update
sudo apt install python3-pip
```

---

## 2. Clone and Install F1TENTH Gym

Navigate to your home directory (or any location of your choice) and clone the F1TENTH Gym repository:

```bash
cd $HOME
git clone https://github.com/f1tenth/f1tenth_gym
cd f1tenth_gym && pip3 install -e .
```

> **Tip:** Installing in **editable mode** (`-e`) allows you to update the package locally without reinstalling it.

---

## 3. Set Up Your ROS2 Workspace

Create a new workspace for your ROS2 simulation:

```bash
cd $HOME
mkdir -p sim_ws/src
```

Navigate into the `src` directory and clone the ROS2 communication bridge repository:

```bash
cd $HOME/sim_ws/src
git clone https://github.com/f1tenth/f1tenth_gym_ros
```

---

## 4. Configure the Simulation

The F1TENTH Gym simulation uses a **YAML configuration file** to define parameters such as the map file path and the number of agents. Here’s how to update it:

1. Open the `sim.yaml` configuration file (located under `f1tenth_gym_ros/config/`) using your preferred text editor:

   ```bash
   nano $HOME/sim_ws/src/f1tenth_gym_ros/config/sim.yaml
   ```

2. Locate the `map_path` parameter and update it with the full path to the map file on your system:

   ```yaml
   map_path: "<your_home_dir>/sim_ws/src/f1tenth_gym_ros/maps/levine"
   ```

   Replace `<your_home_dir>` with your absolute home directory path. For example:

   ```yaml
       map_path: '/home/israel/sim_ws/src/f1tenth_gym_ros/maps/levine'
   ```

   > **Tip:** You can use `pwd` to output the path of your current directory.

---

## 5. Initialize rosdep and Install ROS2 Dependencies

### Initialize rosdep

First, initialize **rosdep** (if not already initialized):

```bash
sudo rosdep init
rosdep update --include-eol-distros
```

### Install ROS2 dependencies

Move to your ROS2 workspace directory and install dependencies using **rosdep**. The command differs based on the ROS2 distribution:

```bash
cd $HOME/sim_ws
rosdep install -i --from-path src --rosdistro humble -y
```

> **Tip:** Ensure that you have sourced your ROS2 environment setup before running `rosdep`:
> ```bash
> source /opt/ros/humble/setup.bash
> ```

---

## 6. Build the ROS2 Workspace

Once dependencies are installed, build your workspace with **colcon**:

```bash
colcon build
```

---

## 7. Launch the Simulation

After building the workspace, launch the simulation. Open a new terminal (or use a terminal multiplexer like **tmux**) and source both your ROS2 setup and local workspace setup:

```bash
source /opt/ros/humble/setup.bash
source $HOME/sim_ws/install/local_setup.bash
```

Now, launch the simulation bridge with:

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

You should see the simulation window (or **RViz** visualization) displaying the simulation environment.

### Temporary fix for gym_bridge crashing

***If you do not see the vehicle, the laser scan points, and keyboard teleop won't work either, run the following command:***

```
python3 -m pip install --user -U "coverage>=7.6.1"
```
> might need to wait a few seconds for everything to load
---

## 8. Configuring the Simulation

The simulation configuration is managed through the `sim.yaml` file, located at `f1tenth_gym_ros/config/sim.yaml`. Here are the key parameters you can adjust:

- **`map_path`:** The full path to your map file (ROS convention: map image and YAML file should have the same name and be in the same directory).
- **`num_agent`:** Set this to `1` for a single agent or `2` for a two-agent race.
- **Starting Pose:** Adjust the `ego` and `opponent` starting poses, defined in the global map coordinate frame.

After making changes to the configuration, rebuild the workspace with `colcon build` to reflect these changes in the container.

---

## 9. Some Topics Published by the Simulation

### In Single-Agent Mode (Topics Published)

- **`/scan`:** The ego agent's laser scan.
- **`/ego_racecar/odom`:** The ego agent's odometry.
- **`/map`:** The map of the environment.

A **TF tree** is maintained for transformation frames.

### In Two-Agent Mode (Topics Published)

In addition to the topics above, these additional topics are published:

- **`/opp_scan`:** The opponent agent's laser scan.
- **`/ego_racecar/opp_odom`:** The opponent agent's odometry (for ego agent's planner).
- **`/opp_racecar/odom`:** The opponent agent's odometry.
- **`/opp_racecar/opp_odom`:** The ego agent's odometry (for opponent agent's planner).

---

## 10. Topics Subscribed by the Simulation

### In Single-Agent Mode (Topics Subscribed)

- **`/drive`:** The ego agent's drive command (via `AckermannDriveStamped` messages).
- **`/initialpose`:** The topic for resetting the ego's pose via RViz's 2D Pose Estimate tool.

### In Two-Agent Mode (Topics Subscribed)

- In addition to the topics listed above, the following topics are available:
  - **`/opp_drive`:** The opponent agent's drive command (via `AckermannDriveStamped` messages). You need to publish to both the ego’s and opponent’s drive topics for both vehicles to move.
  - **`/goal_pose`:** The topic for resetting the opponent agent's pose via RViz's 2D Goal Pose tool.

---

## 11. Enabling Keyboard Teleoperation

The **teleop_twist_keyboard** package is included as part of the simulation's dependencies. To enable keyboard teleoperation, set `kb_teleop` to `True` in the `sim.yaml` configuration file.

After launching the simulation, in another terminal, run:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

In the terminal window running the **teleop** node:

- Press `i` to move forward.
- Press `u` and `o` to move forward and turn.
- Press `,` to move backward.
- Press `m` and `.` to move backward and turn.
- Press `k` to stop.

---

## 12. Developing Your Own Agent in ROS2

You have two options to develop and launch your own agent to control the vehicles:

1. **Create a new package in the existing ROS2 workspace:**  
   Create your package in the `/sim_ws` workspace. After launching the simulation, launch your agent node in another terminal.

2. **Create a new container for your agent node:**  
   Create a separate workspace for your agent node. Ensure both the simulation and agent workspace are on the same network to enable communication.

---
