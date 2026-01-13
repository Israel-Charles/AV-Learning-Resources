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
ros2 interface show /turtle1/cmd_vel
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
