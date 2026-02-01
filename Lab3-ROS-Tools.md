# Lab 3: ROS2 Tools with TurtleSim

## Lab Overview
In this lab, you will explore essential ROS2 tools for visualization, debugging, and data recording using TurtleSim as a demonstration platform. You'll learn to use rqt for graphical interfaces, tf2 for coordinate transformations, RViz2 for 3D visualization, and rosbag2 for data recording and playback.

---

## Learning Objectives
By the end of this lab, you will be able to:
1. Use TurtleSim to understand basic ROS2 concepts
2. Utilize rqt tools for system introspection and control
3. Work with tf2 (Transform Library) to visualize coordinate frames
4. Use RViz2 for 3D visualization of robotic systems
5. Record and replay robot data using rosbag2
6. Understand how multiple ROS tools work together in a live system

---

## Prerequisites

Ensure ROS2 Humble is installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

Install required packages (they should have been already installed from previous labs):

```bash
sudo apt update
sudo apt install ros-humble-turtlesim
sudo apt install ros-humble-rqt*
sudo apt install ros-humble-tf2-tools
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-rosbag2*
```

---

## Part 1: Introduction to TurtleSim

### What is TurtleSim?

TurtleSim is a lightweight simulator for learning ROS concepts. It provides a simple 2D environment with a turtle that can be controlled, making it perfect for understanding publishers, subscribers, services, and parameters.

### Step 1.1: Launch TurtleSim

Open a terminal and start the TurtleSim node:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
```

**Expected Result:** A blue window appears with a turtle in the center.

### Step 1.2: Control the Turtle with Keyboard

Open a **second terminal** and run the teleop (teleoperation) node:

```bash
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtle_teleop_key
```

**Instructions:**
- Keep the second terminal **focused** (clicked)
- Use arrow keys to move the turtle
- The turtle draws a line as it moves

**Experiment:** Try moving the turtle in different patterns (circle, square, zigzag)

### Step 1.3: Explore TurtleSim Topics

Open a **third terminal** and list active topics:

```bash
ros2 topic list
```

**Expected Output:**
```
/parameter_events
/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```

**Key Topics:**
- `/turtle1/cmd_vel`: Velocity commands to control the turtle
- `/turtle1/pose`: Current position and orientation of the turtle
- `/turtle1/color_sensor`: Color information at turtle's location

**Other System Topics**
- `/parameter_events`: This topic is used by ROS 2 to publish notifications when parameters change on any node.
- `/rosout`: This topic is used for logging (INFO, WARN, ERROR, DEBUG, FATAL) from ROS 2 nodes.


### Step 1.4: View Topic Data

**Monitor turtle's position:**
```bash
ros2 topic echo /turtle1/pose
```

**Expected Output:**
```
x: 5.544444561004639
y: 5.544444561004639
theta: 0.0
linear_velocity: 0.0
angular_velocity: 0.0
---
```

Move the turtle and observe how the values change.

**Check topic information:**
```bash
ros2 topic info /turtle1/cmd_vel
```

**Expected Output:**
```
Type: geometry_msgs/msg/Twist
Publisher count: 1
Subscription count: 1
```

**Check message structure:**
```bash
ros2 interface show geometry_msgs/msg/Twist
```

**Expected Output:**
```
# This expresses velocity in free space broken into its linear and angular parts.

Vector3  linear
	float64 x
	float64 y
	float64 z
Vector3  angular
	float64 x
	float64 y
	float64 z
```


### Step 1.5: Control Turtle Programmatically

Publish a velocity command directly:

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

> *Publishes a single velocity command telling turtle1 to move forward at 2.0 m/s while rotating at 1.8 rad/s.* 
> - `ros2 topic pub`: This tells ROS 2 to publish a message on a topic from the command line.
> - `--once`: Publish exactly one message, then exit. Without this flag, the command would publish continuously (default ~1 Hz) and keep running until something stops it (ex: with Ctrl+C).
> - `/turtle1/cmd_vel`: This is the topic name to send the command to.
> - `geometry_msgs/msg/Twist`: This is the message type required by the topic.
> 
> ```
> {
>   linear:  {x: 2.0, y: 0.0, z: 0.0},
>   angular: {x: 0.0, y: 0.0, z: 1.8}
> }
> ```


**Result:** The turtle moves forward while rotating.

**Try continuous publishing:**
```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```
*Publishing at 1 Hz*

Press `Ctrl+C` to stop.

### Step 1.6: Explore TurtleSim Services

** Services in ROS** are used when you want a request–response interaction. Think of a service like a function call over the network. A service is a synchronous communication (request → response) between a client and a server, used for one-time actions, not continuous data flow. This is different from topics, which are publish/subscribe and continuous.

**List available services:**
```bash
ros2 service list
```

**Expected Output:**
```
/clear
/kill
/reset
/spawn
/turtle1/set_pen
/turtle1/teleport_absolute
/turtle1/teleport_relative
```

**Clear the drawing:**
```bash
ros2 service call /clear std_srvs/srv/Empty
```
> - `ros2 service call`: Call a service and wait for its response.
> - `/clear`: This is the service name.
> - `std_srvs/srv/Empty`: This is the service type.
> 
> In this case the service type is empty, no parameter is being sent or received
>
> Note that, just like **message structures** `.msg`, there are **services structures** `.srv`
>
> A service has two parts, separated by ---:
> ```
> # Request
> <fields>
> ---
> # Response
> <fields>
> ```
> 
> Example: std_srvs/srv/Empty
> ```
>
> ---
>
> ```
>
> Example: turtlesim/srv/Spawn
> ```
> float32 x
> float32 y
> float32 theta
> string name
> ---
> string name
> ```

**Spawn a second turtle:**
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

** Run the teleop (teleoperation) node and remap the control topic to control turtle2:**
```
ros2 run turtlesim turtle_teleop_key --ros-args -r /turtle1/cmd_vel:=/turtle2/cmd_vel
```

> --ros-args → says that this is ROS-specific arguments (not regular Linux args)
> -r → remap
> /turtle1/cmd_vel → original topic the node thinks it’s publishing to
> :=
> /turtle2/cmd_vel → new topic it will actually publish to

**Change pen color:**
```bash
ros2 service call /turtle1/set_pen turtlesim/srv/SetPen "{r: 255, g: 0, b: 0, width: 3, 'off': 0}"
```

Now when turtle1 moves, it draws a red line.

**Teleport turtle:**
```bash
ros2 service call /turtle1/teleport_absolute turtlesim/srv/TeleportAbsolute "{x: 5.5, y: 5.5, theta: 0.0}"
```

---

## Part 2: Using `rqt` Tools

### What is `rqt`?

`rqt` is a Qt-based framework for GUI development in ROS. It provides various plugins for visualizing and interacting with ROS systems.

### Step 2.1: Launch rqt

```bash
ros2 run rqt_gui rqt_gui
```

**Alternative (shorter command):**
```bash
rqt
```

### Step 2.2: rqt_graph - Visualize Node Connections

**In the rqt window:**
1. Go to: **Plugins → Introspection → Node Graph**
2. Observe the visual representation of nodes and topics

**What you should see:**
- Nodes: `/turtlesim` and `/teleop_turtle` (if running)
- Topics: `/turtle1/cmd_vel` connecting them
- Arrows showing data flow direction

**Experiment:**
- Uncheck "Hide: Dead sinks" and "Hide: Leaf topics"
- Click "Refresh" to update the graph
- Observe how the graph changes when you start/stop nodes

### Step 2.3: rqt_plot - Plot Topic Data

**In rqt:**
1. Go to: **Plugins → Visualization → Plot**
2. In the topic field, type: `/turtle1/pose/x`
3. Click the **+** button (or press Enter)
4. Add `/turtle1/pose/y` similarly

**Now move the turtle using teleop and observe:**
- Real-time plotting of x and y positions
- How the graphs change based on movement patterns

**Try plotting velocity:**
- Add `/turtle1/pose/linear_velocity`
- Add `/turtle1/pose/angular_velocity`
- Observe how these change when you control the turtle

### Step 2.4: rqt_console - View Log Messages

**In rqt:**
1. Go to: **Plugins → Logging → Console**

**Generate some messages:**

In a terminal, make the turtle hit the wall:
```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 10.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**In rqt_console, you should see:**
- Warning messages about the turtle hitting boundaries
- Different log levels (Debug, Info, Warn, Error, Fatal)

**Experiment with filters:**
- Filter by severity level
- Filter by node name
- Search for specific keywords

### Step 2.5: rqt_publisher - Publish Messages via GUI

**In rqt:**
1. Go to: **Plugins → Topics → Message Publisher**
2. Click the **+** button
3. Select `/turtle1/cmd_vel` from the dropdown
4. Expand the message structure
5. Set values:
   - `linear.x`: 1.0
   - `angular.z`: 0.5
6. Set publishing rate: 10 Hz
7. Check the checkbox to start publishing

**Result:** The turtle moves in a circular pattern.

### Step 2.6: rqt_service_caller - Call Services via GUI

**In rqt:**
1. Go to: **Plugins → Services → Service Caller**
2. Select `/turtle1/set_pen` from dropdown
3. Set parameters:
   - `r`: 0
   - `g`: 255
   - `b`: 0
   - `width`: 5
4. Click **Call**

**Result:** The turtle now draws with a green, thicker line.

**Try other services:**
- `/clear` - Clear the drawing
- `/spawn` - Create another turtle
- `/turtle1/teleport_absolute` - Move turtle to specific position

### Step 2.7: rqt Perspective - Save Your Layout

**After setting up multiple plugins:**
1. Go to: **Perspectives → Create Perspective**
2. Name it: "TurtleSim Debug"
3. Next time you open rqt:
   - **Perspectives → TurtleSim Debug** (loads your saved layout)

---

## Part 3: Working with `tf2` (Transforms)

### What is `tf2`?

`tf2` is ROS2's transform library. It manages coordinate frames and transformations between them, essential for robots with multiple sensors and moving parts.

### Step 3.1: Understanding TurtleSim's Coordinate Frame

TurtleSim creates a transform from `world` to `turtle1` (and any other spawned turtles).

**View available transforms:**
```bash
ros2 run tf2_ros tf2_echo world turtle1
```

**Expected Output:**
```
At time 1735411234.567
- Translation: [5.544, 5.544, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
- Rotation: in RPY (radian) [0.000, 0.000, 0.000]
- Rotation: in RPY (degree) [0.000, 0.000, 0.000]
```

Move the turtle and observe how values update in real-time.

### Step 3.2: Visualize Transform Tree

**Generate a PDF of the transform tree:**
```bash
ros2 run tf2_tools view_frames
```

**Result:** Creates a file named `frames_YYYY-MM-DD_HH-MM-SS.pdf`

**View the PDF:**
```bash
evince frames_*.pdf
```

**What you should see:**
- `world` frame at the root
- `turtle1` frame as a child
- If you spawned `turtle2`, it will also appear

### Step 3.3: Spawn Multiple Turtles and Observe Transforms

**Spawn a second turtle:**
```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 8.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

**View transform between turtles:**
```bash
ros2 run tf2_ros tf2_echo turtle1 turtle2
```

**Expected Output:**
```
At time 1735411234.567
- Translation: [2.456, -3.544, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

This shows the relative position and orientation between turtle1 and turtle2.

### Step 3.4: Monitor All Transforms

**List all active transforms:**
```bash
ros2 run tf2_ros tf2_monitor
```

**Expected Output:**
```
Frame: turtle1 published by: /turtlesim
Average rate: 62.500 Hz
Most recent transform: 0.016 sec old
Buffer length: 4.960 sec

Frame: turtle2 published by: /turtlesim
Average rate: 62.500 Hz
Most recent transform: 0.016 sec old
Buffer length: 4.960 sec
```

### Step 3.5: Understanding Transform Broadcasting

TurtleSim automatically broadcasts transforms. In real robots, you need to:
- Configure URDF (robot description) files
- Set up static transforms for fixed sensors
- Broadcast dynamic transforms for moving parts

**View transform topics:**
```bash
ros2 topic list | grep tf
```

**Expected Output:**
```
/tf
/tf_static
```

**Echo transform messages:**
```bash
ros2 topic echo /tf
```

Move the turtle to see transform updates in real-time.

---

## Part 4: Visualization with `RViz2`

### What is `RViz2`?

`RViz2` (ROS Visualization 2) is a 3D visualization tool for displaying sensor data, robot models, and planning results.

### Step 4.1: Launch RViz2

```bash
ros2 run rviz2 rviz2
```

**Expected Result:** RViz2 window opens (initially empty with a grid).

### Step 4.2: Configure Fixed Frame

**In RViz2:**
1. In the left panel under "Global Options"
2. Change "Fixed Frame" from "map" to "world"
3. If "world" is not available, type it manually

### Step 4.3: Add TF Display

**To visualize coordinate frames:**
1. Click **Add** button (bottom left)
2. Select **TF** from the "By display type" tab
3. Click **OK**

**What you should see:**
- Coordinate frame axes for `world`, `turtle1`, and `turtle2` (if spawned)
- Red = X axis, Green = Y axis, Blue = Z axis

**Experiment:**
- Move turtle1 using teleop
- Watch the `turtle1` frame move in RViz2
- Expand "TF" in the left panel to see individual frames
- Toggle frame labels on/off

### Step 4.4: Customize TF Display

**In the TF display settings (left panel):**
- **Show Names:** Check/uncheck to show frame labels
- **Show Axes:** Check/uncheck to show axis arrows
- **Show Arrow:** Check/uncheck to show relationship arrows
- **Frame Timeout:** Adjust how long frames persist
- **Update Interval:** Control refresh rate

**Adjust individual frames:**
- Expand "Frames" under TF
- Select `turtle1`
- Change marker scale for visibility

### Step 4.5: Add Odometry Display (Turtle Path)

Unfortunately, TurtleSim doesn't publish standard odometry. But we can visualize the pose.

**Add Pose Display:**
1. Click **Add**
2. Go to **By topic** tab
3. Find `/turtle1/pose`
4. Select **Pose**
5. Click **OK**

**Configure the Pose display:**
- Shape: Arrow
- Color: Choose a visible color (e.g., red)
- Shaft/Head Length/Radius: Adjust for visibility

**Result:** An arrow shows the turtle's current position and heading.

### Step 4.6: Visualize Multiple Turtles

**If you have turtle2 spawned:**
1. Add another Pose display
2. Topic: `/turtle2/pose`
3. Use a different color (e.g., blue)

**Now you can see:**
- Both turtles' positions and orientations
- Their coordinate frames
- Relative positioning

### Step 4.7: Adjust View in RViz2

**Camera controls:**
- **Left mouse button:** Rotate view
- **Middle mouse button:** Pan view
- **Scroll wheel:** Zoom in/out
- **Shift + Left mouse:** Pan (alternative)

**Switch view types:**
1. Click "Views" in left panel
2. Select "Type" dropdown
3. Try different view types:
   - **Orbit:** Rotate around a point (default)
   - **XY Orbit:** Fixed Z axis
   - **TopDownOrtho:** 2D top-down view (best for TurtleSim)
   - **FPS:** First-person view

**For TurtleSim, recommend:**
- Type: TopDownOrtho
- Adjust distance to see entire window

### Step 4.8: Save RViz2 Configuration

**Save your configuration:**
1. File → **Save Config As**
2. Save as: `~/ros2_ws/turtlesim_config.rviz`

**Load configuration next time:**
```bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/turtlesim_config.rviz
```

---

## Part 5: Recording and Playing Back Data with rosbag2

### What is rosbag2?

rosbag2 records and plays back ROS2 topics, allowing you to:
- Record sensor data for offline analysis
- Replay scenarios for testing
- Share data with others
- Debug issues by replaying exact conditions

### Step 5.1: Record Data from TurtleSim

**Ensure TurtleSim is running, then record data:**

```bash
cd ~
mkdir -p ros2_bags
cd ros2_bags
ros2 bag record /turtle1/cmd_vel /turtle1/pose
```

**Expected Output:**
```
[INFO] [rosbag2_storage]: Opened database 'rosbag2_YYYY_MM_DD-HH_MM_SS'.
[INFO] [rosbag2_transport]: Listening for topics...
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/cmd_vel'
[INFO] [rosbag2_transport]: Subscribed to topic '/turtle1/pose'
[INFO] [rosbag2_transport]: All requested topics are subscribed. Stopping discovery...
```

**Now, move the turtle using teleop for 20-30 seconds.**

**Stop recording:** Press `Ctrl+C`

**Output:**
```
[INFO] [rosbag2_storage]: Closed database 'rosbag2_YYYY_MM_DD-HH_MM_SS'.
```

### Step 5.2: Inspect the Recorded Bag

**View bag information:**
```bash
ros2 bag info rosbag2_YYYY_MM_DD-HH_MM_SS
```

**Expected Output:**
```
Files:             rosbag2_YYYY_MM_DD-HH_MM_SS_0.db3
Bag size:          48.2 KiB
Storage id:        sqlite3
Duration:          25.123s
Start:             Jan 13 2026 10:30:45.123 (1735411845.123)
End:               Jan 13 2026 10:31:10.246 (1735411870.246)
Messages:          1567
Topic information:
  Topic: /turtle1/cmd_vel | Type: geometry_msgs/msg/Twist | Count: 245 | Serialization Format: cdr
  Topic: /turtle1/pose | Type: turtlesim/msg/Pose | Count: 1322 | Serialization Format: cdr
```

### Step 5.3: Play Back Recorded Data

**First, close TurtleSim and teleop (or open in new terminal).**

**Clear the turtle's path:**
```bash
ros2 service call /clear std_srvs/srv/Empty
```

**Play back the recording:**
```bash
ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS
```

**Expected Output:**
```
[INFO] [rosbag2_player]: Opened database 'rosbag2_YYYY_MM_DD-HH_MM_SS'.
[INFO] [rosbag2_player]: Playback duration: 25.123s
[INFO] [rosbag2_player]: Playback until timestamp: ...
```

**Result:** The turtle replays your exact movements!

### Step 5.4: Play Back at Different Speeds

**Play at half speed:**
```bash
ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS --rate 0.5
```

**Play at double speed:**
```bash
ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS --rate 2.0
```

**Play in a loop:**
```bash
ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS --loop
```

Press `Ctrl+C` to stop.

### Step 5.5: Record All Topics

**Record everything:**
```bash
ros2 bag record -a
```

**Warning:** This records ALL topics and can generate large files quickly. Use with caution.

**Better approach - record specific topics:**
```bash
ros2 bag record /turtle1/cmd_vel /turtle1/pose /turtle1/color_sensor
```

### Step 5.6: Record with Custom Bag Name

**Specify output directory and name:**
```bash
ros2 bag record -o my_turtle_recording /turtle1/cmd_vel /turtle1/pose
```

**Result:** Creates a bag named `my_turtle_recording` instead of timestamped name.

### Step 5.7: Advanced Playback Options

**Start playback from specific time:**
```bash
ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS --start-offset 5.0
```
(Starts 5 seconds into the recording)

**Play only a portion:**
```bash
ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS --duration 10.0
```
(Plays only first 10 seconds)

### Step 5.8: Export Bag Data

**List topics in bag:**
```bash
ros2 bag info rosbag2_YYYY_MM_DD-HH_MM_SS
```

**Play and echo to terminal:**
```bash
# Terminal 1: Play bag
ros2 bag play rosbag2_YYYY_MM_DD-HH_MM_SS

# Terminal 2: Echo topic
ros2 topic echo /turtle1/pose
```

---

## Lab Summary and Key Takeaways

### What We Learned:

1. **TurtleSim**: Basic ROS2 concepts with a simple simulator
2. **rqt**: Graphical tools for debugging and visualization
   - rqt_graph: System architecture
   - rqt_plot: Real-time data plotting
   - rqt_console: Log message viewing
   - rqt_publisher/service_caller: Interaction tools

3. **tf2**: Transform management and coordinate frames
   - Understanding frame relationships
   - Visualizing transform trees
   - Monitoring transform updates

4. **RViz2**: 3D visualization
   - Displaying coordinate frames
   - Visualizing robot poses
   - Customizing displays

5. **rosbag2**: Data recording and playback
   - Recording topics
   - Playing back data
   - Analyzing recorded information

### These skills are essential for:
- Debugging autonomous racing robots
- Understanding sensor coordinate frames
- Visualizing robot state and perception
- Recording and analyzing test runs
- Developing and testing algorithms offline

---

## Common Issues and Solutions

**Issue:** RViz doesn't show TF frames
- **Solution:** Check that Fixed Frame is set to "world"
- Verify TurtleSim is running and publishing transforms

**Issue:** Rosbag recording fails
- **Solution:** Check you have write permissions in target directory
- Ensure disk space is available
- Verify topic names are correct

**Issue:** Launch file crashes
- **Solution:** Check syntax carefully (Python indentation)
- Ensure all imports are included
- Test each node individually first

**Issue:** Playback doesn't work
- **Solution:** Verify bag file path is correct
- Ensure TurtleSim is running before playback
- Check that bag contains expected topics

---

# Deliverable

## Integrated TurtleSim Visualization and Recording System

### Objective
Create a comprehensive ROS2 launch file that integrates all visualization and debugging tools learned in this lab to demonstrate a complete robotic system workflow: control, visualization, recording, and playback.

---

### Overview

You will create a launch file that orchestrates a demonstration of TurtleSim with the following sequence:

1. **System Startup:** All required nodes and visualization tools launch automatically
2. **Live Operation Phase (~20 seconds):** User controls turtle1 with keyboard while:
   - RViz2 displays turtle1's movement relative to turtle2
   - rqt shows real-time plots of turtle1's position/velocity
   - rqt displays tf transform information (turtle1 → world)
   - rosbag2 records keyboard commands
3. **Playback Phase:** After recording stops, rosbag automatically replays commands at 2x speed while all visualization tools continue displaying the data

---

### Detailed Requirements

#### 1. Launch File Specifications

Create a launchfile that launches:

**Core Nodes:**
- TurtleSim node
- Keyboard teleoperation node for turtle1
- A second turtle (turtle2) spawned at a specific location

**Visualization Tools:**
- **rqt with at least two configurations:**
  - **Window 1:** Plot display showing at least one turtle1 metric (pose/x, pose/y, linear_velocity, or angular_velocity, etc...)
  - **Window 2:** TF tree or echo display showing transform from turtle1 with respect to world frame
  
- **RViz2** configured to display:
  - Both turtle1 and turtle2 coordinate frames
  - Visual representation showing turtle1's movement relative to turtle2
  - Appropriate fixed frame and display settings

**Recording and Playback:**
- **rosbag2** that:
  - Records `/turtle1/cmd_vel` (keyboard commands) for approximately 20 seconds
  - Automatically stops recording after the time limit
  - Automatically replays the recorded commands at 2x speed
  - Keeps all visualization windows active during both recording and playback phases

#### 2. Implementation Options

You may choose **either** approach:

**Option A: Pre-configured Files**
- Create and save RViz configuration file (ex: `turtlesim_demo.rviz`)
- Create and save rqt perspective files for both windows
- Launch file loads these configurations

**Option B: Programmatic Configuration**
- Use launch file commands/arguments to configure rqt and RViz
- Can use `ExecuteProcess` with command-line arguments to set up displays
- No separate configuration files needed

#### 3. User Experience Requirements

The system must provide clear feedback:
- Terminal messages or something obvious that indicates current phase (Recording/Playback)
- Countdown or timer showing, anything that shows recording progress
- Clear indication when playback begins
- All visualisation windows remain active throughout the entire demonstration
- **System should run autonomously** after initial launch (minimal user intervention needed except for controlling turtle1)

---

### Workflow Summary

```
[Launch] → [Spawn turtle2] → [Open visualizations] → [User instructions] →
[Start recording] → [User drives turtle1 for 20s] → [Stop recording] →
[Playback at 2x speed] → [Complete]
```

### **Submission Deliverables**

1. **Submit only the `src` folder** from your ROS 2 workspace.

   * All of your work **must be contained inside this `src` folder**.
   * Do **not** include `build/`, `install/`, or `log/` directories.

2. **Make sure all required packages, nodes, launch files, and config files** needed to run your assignment are inside the `src` folder.

3. **Include a `README.md` file inside the `src` folder** that clearly explains:

   * What you implemented for the assignment
   * How to build the workspace
   * How to run your code (exact commands)

4. **Zip the `src` folder only** (not the full workspace).

5. **Name the ZIP file exactly as follows:**

   ```
   first-name_last-name_studentID_lab#.zip
   ```

   Example:

   ```
   jane_doe_1234567_lab2.zip
   ```

---

## **Grading Rubric (100 Points)**

| Category                           | Points |
| ---------------------------------- | ------ |
| Fully functioning launch file | 30     |
| Proper `rviz2` usage      | 15     |
| Proper `rqt` usage    | 15     |
| Proper `tf2` usage      | 15     |
| Proper `rosbag` usage      | 15     |
| Well organized submission, well formatted codes          | 10     |
