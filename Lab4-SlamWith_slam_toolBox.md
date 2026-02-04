# Lab 4: Intro to SLAM with slam_toolbox

## Lab Overview
In this lab, you will learn about Simultaneous Localization and Mapping (SLAM) and gain proficiency with slam_toolbox, one of the most popular SLAM packages in ROS2. You'll work with a pre-recorded rosbag from a race car completing one lap around a track, using the LiDAR scan data and odometry to build a map of the environment.

---

## Learning Objectives
By the end of this lab, you will be able to:
1. Understand the fundamentals of SLAM and its importance in autonomous systems
2. Install and configure slam_toolbox for ROS2 Humble
3. Run slam_toolbox with different configurations and modes
4. Use RViz2 to visualize SLAM in real-time
5. Generate, save, and load maps from SLAM sessions
6. Tune SLAM parameters for different scenarios
7. Analyze SLAM performance and map quality

---

## Prerequisites

Ensure ROS2 Humble is installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

---

## Part 1: Understanding SLAM

### What is SLAM?

**SLAM (Simultaneous Localization and Mapping)** is the computational problem of constructing or updating a map of an unknown environment while simultaneously keeping track of an agent's location within it.

### Why is SLAM Important?

For autonomous racing robots:
- **Map Building:** Create accurate track maps without prior knowledge
- **Localization:** Know precise position on track in real-time
- **Path Planning:** Plan optimal racing lines based on map
- **Obstacle Detection:** Identify and map dynamic obstacles

### SLAM Components

1. **Sensor Input:**
   - LiDAR scans (`/scan`) - Distance measurements in 360°
   - Odometry (`/odom`) - Estimated position from wheel encoders/IMU
   - Transforms (`/tf`) - Coordinate frame relationships

2. **SLAM Algorithm:**
   - Processes sensor data
   - Estimates robot pose (localization)
   - Builds map (mapping)
   - Handles loop closures (recognizing previously visited locations)

3. **Output:**
   - Occupancy grid map (2D representation)
   - Robot pose estimates
   - Updated transforms (map → base_link)

### Types of SLAM

- **Online SLAM:** Real-time mapping while navigating (what we'll do)
- **Offline SLAM:** Process recorded data to create map
- **2D SLAM:** Creates flat, top-down maps (our focus)
- **3D SLAM:** Creates 3D volumetric maps

---

## Part 2: Installing slam_toolbox

### What is slam_toolbox?

slam_toolbox is a comprehensive SLAM solution for ROS2 that provides:
- Multiple SLAM modes (online, offline, localization)
- Loop closure detection
- Map serialization (save/load)
- Interactive map manipulation
- Integration with Nav2 (will see this in later labs)

### Step 2.1: Install slam_toolbox

```bash
sudo apt update
sudo apt install ros-humble-slam-toolbox
```

### Step 2.2: Verify Installation

```bash
ros2 pkg list | grep slam_toolbox
```

**Expected Output:**
```
slam_toolbox
```

### Step 2.3: Check Available Executables

```bash
ros2 pkg executables slam_toolbox
```

**Expected Similar Output:**
```
slam_toolbox async_slam_toolbox_node
slam_toolbox sync_slam_toolbox_node
...
```

### Step 2.4: Explore slam_toolbox Parameters

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml
```

Press `Ctrl+C` to stop. This shows that slam_toolbox requires a configuration file.

---

## Part 3: Preparing the Workspace (Suggestive)

### Step 3.1: Create Working Directory

```bash
mkdir -p ~/slam_lab
cd ~/slam_lab
mkdir -p maps config bags
```

**Potential Directory structure:**
```
~/slam_lab/
├── maps/      # For saving generated maps
├── config/    # For SLAM configuration files
└── bags/      # For rosbag files
```

### Step 3.2: Download/Copy the Rosbag

For this example, we'll assume the rosbag is named `racecar_lap.db3`.

> Note that rosbags can be in different formats. Some can be a singular file and some can be folders. To copy folder, use option `-r` as in `cp -r`

```bash
# Copy rosbag to folder:
cp /path/to/racecar_lap* ~/slam_lab/bags/

# Verify the bag
cd ~/slam_lab/bags
ros2 bag info racecar_lap
```

**Expected Output:**
```
Files:             racecar_lap_0.db3
Bag size:          XXX.X MiB
Storage id:        sqlite3
Duration:          XX.XXs
Start:             ...
End:               ...
Messages:          XXXX
Topic information:
  Topic: /scan | Type: sensor_msgs/msg/LaserScan | Count: XXX
  Topic: /ego_racecar/odom | Type: nav_msgs/msg/Odometry | Count: XXX
  Topic: /tf | Type: tf2_msgs/msg/TFMessage | Count: XXX
  Topic: /tf_static | Type: tf2_msgs/msg/TFMessage | Count: XXX
  Topic: /clock | Type: rosgraph_msgs/msg/Clock | Count: XXX
  Topic: /joint_states | Type: sensor_msgs/msg/JointState | Count: XXX
  Topic: /ego_robot_description | Type: std_msgs/msg/String | Count: XXX
```

### Step 3.3: Inspect the Topics

Let's understand what data we have:

**Play the bag briefly to inspect:**
```bash
ros2 bag play racecar_lap
```

**In another terminal, echo the scan topic:**
```bash
ros2 topic echo /scan --once
```

**Expected Output:**
```
header:
  stamp:
    sec: ...
    nanosec: ...
  frame_id: ego_racecar/laser
angle_min: -3.14159...
angle_max: 3.14159...
angle_increment: 0.00436...
time_increment: 0.0
scan_time: 0.0
range_min: 0.0
range_max: 30.0
ranges:
- 5.234
- 5.189
- ...
intensities: []
```

**Check odometry:**
```bash
ros2 topic echo /ego_racecar/odom --once
```

---

## Part 4: Creating SLAM Configuration Files

### Understanding slam_toolbox Configuration

slam_toolbox uses YAML configuration files to set parameters. Let's explore the default and create our custom configuration.

### Step 4.1: Examine Default Configuration

```bash
cat /opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml
```

> Note that this is the configurations that gets launch by default when you launch the default `slam_toolbox` launchfiles

**Key parameters you'll see:**
- `odom_frame`: Odometry frame ID
- `map_frame`: Map frame ID
- `base_frame`: Robot base frame ID
- `scan_topic`: LiDAR scan topic
- `resolution`: Map resolution (meters per pixel)
- `max_laser_range`: Maximum range to use from LiDAR
- Various tuning parameters for scan matching, loop closure, etc.

### Step 4.2: Create Custom Configuration for Async SLAM

In Async SLAM, data is processed in parallel as it arrives, rather than waiting for synchronized frames, allowing for faster real-time performance

**File:** `~/slam_lab/config/online_async_racecar.yaml`

```yaml
# Online Async SLAM Configuration for Race Car
# This mode continuously builds a map while the robot moves

slam_toolbox:
  ros__parameters:

    # ROS Parameters
    odom_frame: ego_racecar/odom
    map_frame: map
    base_frame: ego_racecar/base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping # mapping, localization, or lifelong
    
    # Debugging
    debug_logging: false
    throttle_scans: 1  # Process every scan (1), or skip scans (>1)
    
    # Transform publication
    transform_publish_period: 0.02  # 50 Hz
    map_update_interval: 5.0        # Update map every 5 seconds
    
    # Resolution of the map (meters per pixel)
    resolution: 0.05  # 5cm resolution - good for racing
    
    # Maximum usable range of the LiDAR
    max_laser_range: 25.0  # meters
    minimum_travel_distance: 0.2   # minimum distance to travel before processing (meters)
    minimum_travel_heading: 0.2    # minimum rotation before processing (radians)
    
    # Scan matching parameters
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.1
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    
    # Loop closure parameters
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.35
    loop_match_minimum_response_fine: 0.45
    
    # Correlation parameters
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Optimization parameters
    optimize_every_n_nodes: 20
    
    # Scan matcher parameters
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

**Save this file.**

### Step 4.3: Create Configuration for Sync SLAM

Sync SLAM processes scans synchronously (waits for each scan to be processed before accepting the next). This is more accurate but slower.

**File:** `~/slam_lab/config/online_sync_racecar.yaml`

```yaml
# Online Sync SLAM Configuration for Race Car
# More accurate but slower processing

slam_toolbox:
  ros__parameters:

    # ROS Parameters
    odom_frame: ego_racecar/odom
    map_frame: map
    base_frame: ego_racecar/base_link
    scan_topic: /scan
    use_map_saver: true
    mode: mapping
    
    # Debugging
    debug_logging: false
    throttle_scans: 1
    
    # Transform publication
    transform_publish_period: 0.02
    map_update_interval: 2.0  # Update more frequently for sync mode
    
    # Resolution
    resolution: 0.05
    
    # Range
    max_laser_range: 25.0
    minimum_travel_distance: 0.15  # Process more frequently
    minimum_travel_heading: 0.15
    
    # Scan matching parameters (more strict for accuracy)
    scan_buffer_size: 20
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.2  # Higher threshold
    link_scan_maximum_distance: 1.5
    loop_search_maximum_distance: 3.0
    
    # Loop closure
    do_loop_closing: true
    loop_match_minimum_chain_size: 10
    loop_match_maximum_variance_coarse: 3.0
    loop_match_minimum_response_coarse: 0.4
    loop_match_minimum_response_fine: 0.5
    
    # Correlation
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Optimization
    optimize_every_n_nodes: 10  # Optimize more frequently
    
    # Scan matcher
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

### Step 4.4: Create Configuration for Localization Mode

Localization mode uses an existing map to localize the robot (doesn't update the map).

**File:** `~/slam_lab/config/localization_racecar.yaml`

```yaml
# Localization Mode - Use existing map to localize robot

slam_toolbox:
  ros__parameters:

    # ROS Parameters
    odom_frame: ego_racecar/odom
    map_frame: map
    base_frame: ego_racecar/base_link
    scan_topic: /scan
    use_map_saver: false  # Don't save map in localization mode
    mode: localization
    
    # Map to load (will be set when loading)
    map_file_name: ""
    map_start_at_dock: true
    
    # Debugging
    debug_logging: false
    throttle_scans: 1
    
    # Transform publication
    transform_publish_period: 0.02
    
    # Resolution (must match saved map)
    resolution: 0.05
    
    # Range
    max_laser_range: 25.0
    minimum_travel_distance: 0.2
    minimum_travel_heading: 0.2
    
    # Scan matching (tuned for localization)
    scan_buffer_size: 10
    scan_buffer_maximum_scan_distance: 10.0
    link_match_minimum_response_fine: 0.3
    link_scan_maximum_distance: 1.5
    
    # No loop closure in localization mode
    do_loop_closing: false
    
    # Correlation
    correlation_search_space_dimension: 0.5
    correlation_search_space_resolution: 0.01
    correlation_search_space_smear_deviation: 0.1
    
    # Scan matcher
    coarse_search_angle_offset: 0.349
    coarse_angle_resolution: 0.0349
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
    use_response_expansion: true
```

---


