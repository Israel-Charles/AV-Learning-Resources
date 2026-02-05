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

## Part 5: Running SLAM with Async Mode

### Step 5.1: Setup RViz Configuration

First, let's create an RViz config for SLAM visualization.

**File:** `~/slam_lab/config/slam_visualization.rviz`

```yaml
Panels:
  - Class: rviz_common/Displays
    Help Height: 78
    Name: Displays
    Property Tree Widget:
      Expanded:
        - /Global Options1
        - /Map1
        - /LaserScan1
      Splitter Ratio: 0.5
    Tree Height: 549
  - Class: rviz_common/Views
    Expanded:
      - /Current View1
    Name: Views
    Splitter Ratio: 0.5
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 0.5
      Cell Size: 1
      Class: rviz_default_plugins/Grid
      Color: 160; 160; 164
      Enabled: true
      Line Style:
        Line Width: 0.029999999329447746
        Value: Lines
      Name: Grid
      Normal Cell Count: 0
      Offset:
        X: 0
        Y: 0
        Z: 0
      Plane: XY
      Plane Cell Count: 100
      Reference Frame: <Fixed Frame>
      Value: true
    
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: false
        ego_racecar/base_link:
          Value: true
        ego_racecar/laser:
          Value: true
        map:
          Value: true
      Marker Scale: 0.5
      Name: TF
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Tree:
        map:
          ego_racecar/base_link:
            ego_racecar/laser:
              {}
      Update Interval: 0
      Value: true
    
    - Alpha: 0.7
      Class: rviz_default_plugins/Map
      Color Scheme: map
      Draw Behind: false
      Enabled: true
      Name: Map
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /map_updates
      Use Timestamp: false
      Value: true
    
    - Alpha: 1
      Autocompute Intensity Bounds: true
      Autocompute Value Bounds:
        Max Value: 10
        Min Value: -10
        Value: true
      Axis: Z
      Channel Name: intensity
      Class: rviz_default_plugins/LaserScan
      Color: 255; 0; 0
      Color Transformer: FlatColor
      Decay Time: 0
      Enabled: true
      Invert Rainbow: false
      Max Color: 255; 255; 255
      Max Intensity: 0
      Min Color: 0; 0; 0
      Min Intensity: 0
      Name: LaserScan
      Position Transformer: XYZ
      Selectable: true
      Size (Pixels): 3
      Size (m): 0.05
      Style: Flat Squares
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /scan
      Use Fixed Frame: true
      Use rainbow: true
      Value: true
    
    - Alpha: 1
      Axes Length: 0.3
      Axes Radius: 0.03
      Class: rviz_default_plugins/PoseWithCovariance
      Color: 255; 25; 0
      Covariance:
        Orientation:
          Alpha: 0.5
          Color: 255; 255; 127
          Color Style: Unique
          Frame: Local
          Offset: 1
          Scale: 1
          Value: true
        Position:
          Alpha: 0.30000001192092896
          Color: 204; 51; 204
          Scale: 1
          Value: true
        Value: true
      Enabled: true
      Head Length: 0.15
      Head Radius: 0.1
      Name: Robot Pose
      Shaft Length: 0.3
      Shaft Radius: 0.05
      Shape: Arrow
      Topic:
        Depth: 5
        Durability Policy: Volatile
        Filter size: 10
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /pose
      Value: true
    
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
      Hide Inactive Objects: true
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
    - Class: rviz_default_plugins/FocusCamera
    - Class: rviz_default_plugins/Measure
      Line color: 128; 128; 0
    - Class: rviz_default_plugins/SetInitialPose
      Covariance x: 0.25
      Covariance y: 0.25
      Covariance yaw: 0.06853891909122467
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /initialpose
    - Class: rviz_default_plugins/SetGoal
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /goal_pose
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 25
      Enable Stereo Rendering:
        Stereo Eye Separation: 0.05999999865889549
        Stereo Focal Distance: 1
        Swap Stereo Eyes: false
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05000000074505806
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.009999999776482582
      Pitch: 1.5697963237762451
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 3.1415927410125732
    Saved: ~
Window Geometry:
  Displays:
    collapsed: false
  Height: 846
  Hide Left Dock: false
  Hide Right Dock: false
  QMainWindow State: 000000ff00000000fd000000040000000000000156000002b0fc0200000008fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d000002b0000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261000000010000010f000002b0fc0200000003fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000000a00560069006500770073010000003d000002b0000000a400fffffffb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004b00000003efc0100000002fb0000000800540069006d00650100000000000004b0000002fb00fffffffb0000000800540069006d006501000000000000045000000000000000000000023f000002b000000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
  Selection:
    collapsed: false
  Tool Properties:
    collapsed: false
  Views:
    collapsed: false
  Width: 1200
  X: 0
  Y: 0
```

### Step 5.2: Launch Async SLAM

Open **Terminal 1** - Launch RViz:
```bash
cd ~/slam_lab
source /opt/ros/humble/setup.bash
rviz2 -d config/slam_visualization.rviz
```

Open **Terminal 2** - Launch slam_toolbox:
```bash
cd ~/slam_lab
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py \
  params_file:=config/online_async_racecar.yaml \
  use_sim_time:=true
```

**Expected Output:**
```
[INFO] [async_slam_toolbox_node]: Node created
[INFO] [async_slam_toolbox_node]: Using solver plugin: solver_plugins::CeresSolver
[INFO] [async_slam_toolbox_node]: CeresSolver: Using SPARSE_NORMAL_CHOLESKY linear algebra.
```

Open **Terminal 3** - Play the rosbag:
```bash
cd ~/slam_lab/bags
source /opt/ros/humble/setup.bash
ros2 bag play racecar_lap --clock
```

**What to Observe in RViz:**
1. **Map gradually appears** as the car drives
2. **Red laser scan points** showing LiDAR data
3. **TF frames** moving as the car navigates
4. **Map updates** showing walls and obstacles

### Step 5.3: Monitor SLAM Performance

Open **Terminal 4** - Check topics:
```bash
ros2 topic list
```

**Expected Output (new SLAM topics):**
```
/map
/map_metadata
/pose
/slam_toolbox/feedback
/slam_toolbox/graph_visualization
/slam_toolbox/scan_visualization
/slam_toolbox/update_map
```

**Monitor map topic:**
```bash
ros2 topic hz /map
```

**Check slam_toolbox node info:**
```bash
ros2 node info /async_slam_toolbox_node
```

### Step 5.4: Understanding What's Happening

As the bag plays:
1. **SLAM receives** `/scan` (LiDAR data) and `/ego_racecar/odom` (odometry)
2. **Processes scans** to match them with previous scans
3. **Estimates robot pose** by combining odometry and scan matching
4. **Builds occupancy grid map** showing free space (white), obstacles (black), and unknown (gray)
5. **Publishes transform** from `map` to `ego_racecar/base_link`
6. **Detects loop closures** when returning to previously visited areas (optimizes map)

### Step 5.5: Save the Map

Once the bag finishes playing:

**In Terminal 2 (where SLAM is running), press Ctrl+C to stop**

**Or use the map server to save manually:**

Open **Terminal 5**:
```bash
cd ~/slam_lab/maps
ros2 run nav2_map_server map_saver_cli -f racetrack_async_map
```

**Expected Output:**
```
[INFO] [map_saver]: Saving map to 'racetrack_async_map.pgm' and 'racetrack_async_map.yaml'
[INFO] [map_saver]: Map saved
```

**Two files created:**
- `racetrack_async_map.pgm` - The actual map image
- `racetrack_async_map.yaml` - Map metadata (resolution, origin, etc.)

**View the map:**
```bash
eog racetrack_async_map.pgm
```

or

```bash
gimp racetrack_async_map.pgm
```

---

