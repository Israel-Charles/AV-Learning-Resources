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

## Part 1: Understanding Follow the Gap Algorithm

### What is Follow the Gap?

**Follow the Gap** is a reactive obstacle avoidance method that:
- Finds the largest "gap" (free space) in LiDAR data
- Steers toward the center or furthest point in that gap
- Works in real-time without mapping or planning
- Handles dynamic obstacles naturally

### Algorithm Overview

```
1. PREPROCESS: Clean and filter LiDAR data
   ↓
2. FIND CLOSEST POINT: Identify nearest obstacle
   ↓
3. SAFETY BUBBLE: Clear area around closest point
   ↓
4. FIND MAX GAP: Locate largest continuous free space
   ↓
5. FIND BEST POINT: Select target within gap
   ↓
6. ACTUATE: Steer toward target point
```

### Detailed Step Breakdown

#### Step 1: Preprocess LiDAR Data

**Why?** Raw LiDAR data is noisy and may contain outliers.

**Methods:**
- **Windowed averaging**: Smooth data by averaging neighboring points
- **Range limiting**: Ignore points beyond certain distance (e.g., > 3m)
- **Median filtering**: Remove spikes and noise

**Example:**
```python
# Raw ranges: [1.2, 5.5, 1.3, 1.1, 1.4, ...]
# After preprocessing: [1.2, 1.3, 1.3, 1.1, 1.4, ...]
#                       (outlier 5.5 removed/smoothed)
```

#### Step 2: Find Closest Point

**Why?** The closest obstacle is the most dangerous - we must avoid it.

**Method:**
```python
closest_idx = np.argmin(ranges)
closest_distance = ranges[closest_idx]
```

#### Step 3: Create Safety Bubble

**Why?** To ensure we don't get too close to obstacles, we create a "bubble" around the closest point where we won't navigate.

**Method:**
- Calculate bubble radius (e.g., 0.3-0.5 meters)
- Set all points within this bubble to 0 (marking them as obstacles)

**Geometry:**
```
For each point at angle θ and distance r:
- If point is within bubble_radius of closest_point:
  - Set range[i] = 0
```

#### Step 4: Find Maximum Gap

**Why?** The largest gap is the safest and most navigable space.

**Method:**
- Find all sequences of non-zero consecutive elements
- Select the longest sequence
- Return start and end indices

**Example:**
```
ranges: [0, 0, 0, 2.1, 2.3, 2.5, 0, 0, 3.1, 3.2, 3.3, 3.4, 0, 0]
gaps:          [---gap1---]        [------gap2------]
max gap: gap2 (indices 8-11, length 4)
```

#### Step 5: Find Best Point in Gap

**Why?** We need a specific target to steer toward within the gap.

**Strategies:**
1. **Naive**: Choose furthest point in gap
2. **Centered**: Choose middle of gap
3. **Weighted**: Balance between furthest and centered
4. **Advanced**: Consider current velocity and steering constraints

**Example:**
```
Gap: [3.1, 3.2, 3.3, 3.4]
Furthest: 3.4 (index 11)
Centered: Between 3.2 and 3.3 (index 9-10)
```

#### Step 6: Actuate the Car

**Why?** Convert target point to steering angle and velocity.

**Method:**
- Calculate angle to best point
- Set steering angle proportional to this angle
- Adjust velocity based on gap size and steering angle

---

## Part 2: Setting Up the Workspace

### Step 2.1: Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create follow_the_gap --build-type ament_python \
  --dependencies rclpy sensor_msgs ackermann_msgs std_msgs
```

### Step 2.2: Create Directory Structure

```bash
cd follow_the_gap
mkdir -p launch config
```

**Resulting structure:**
```
follow_the_gap/
├── follow_the_gap/
│   ├── __init__.py
│   └── reactive_node.py        # Main implementation (we will create)
├── launch/
│   └── follow_gap.launch.py    # Launch file (we will create)
├── config/
│   └── gap_params.yaml         # Parameters (we will create)
├── package.xml
├── setup.py
└── resource/
```

### Step 2.3: Make sure `package.xml` has the right dependencies

**File:** `~/ros2_ws/src/follow_the_gap/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>follow_the_gap</name>
  <version>1.0.0</version>
  <description>Follow the Gap reactive obstacle avoidance for F1TENTH</description>
  <maintainer email="student@example.com">Student Name</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>ackermann_msgs</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 2.4: Update setup.py to include entry points, config, and launch files

**File:** `~/ros2_ws/src/follow_the_gap/setup.py`

```python
from setuptools import setup
import os
from glob import glob

package_name = 'follow_the_gap'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Name',
    maintainer_email='student@example.com',
    description='Follow the Gap obstacle avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_node = follow_the_gap.reactive_node:main',
        ],
    },
)
```

---

## Part 3: Implementing Follow the Gap - Step by Step

### Step 3.1: Create Basic Node Structure

**File:** `~/ros2_ws/src/follow_the_gap/follow_the_gap/reactive_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class ReactiveFollowGap(Node):
    """ 
    Implement Follow the Gap reactive obstacle avoidance algorithm
    """
    def __init__(self):
        super().__init__('reactive_node')
        
        # Declare parameters
        self.declare_parameter('bubble_radius', 0.3)  # meters
        self.declare_parameter('preprocess_conv_size', 3)  # window size for averaging
        self.declare_parameter('max_lidar_range', 3.0)  # max range to consider (meters)
        self.declare_parameter('speed_min', 0.5)  # minimum speed (m/s)
        self.declare_parameter('speed_max', 2.0)  # maximum speed (m/s)
        self.declare_parameter('steering_gain', 1.0)  # steering proportional gain
        
        # Get parameters
        self.bubble_radius = self.get_parameter('bubble_radius').value
        self.preprocess_conv_size = self.get_parameter('preprocess_conv_size').value
        self.max_lidar_range = self.get_parameter('max_lidar_range').value
        self.speed_min = self.get_parameter('speed_min').value
        self.speed_max = self.get_parameter('speed_max').value
        self.steering_gain = self.get_parameter('steering_gain').value
        
        # Topics
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        
        # Create subscriber to LiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            lidarscan_topic,
            self.lidar_callback,
            10)
        
        # Create publisher to drive
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            drive_topic,
            10)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Follow the Gap Node Initialized')
        self.get_logger().info(f'Bubble Radius: {self.bubble_radius}m')
        self.get_logger().info(f'Max LiDAR Range: {self.max_lidar_range}m')
        self.get_logger().info(f'Speed Range: {self.speed_min}-{self.speed_max} m/s')
        self.get_logger().info('='*50)

    def preprocess_lidar(self, ranges):
        """ 
        Preprocess the LiDAR scan array.
        1. Setting each value to the mean over some window
        2. Rejecting high values (> max_lidar_range)
        
        Args:
            ranges: array of LiDAR ranges
            
        Returns:
            proc_ranges: processed array of ranges
        """
        # TODO: Implement preprocessing
        proc_ranges = np.array(ranges)
        return proc_ranges
    
    def find_max_gap(self, free_space_ranges):
        """ 
        Return the start index & end index of the max gap in free_space_ranges
        
        Args:
            free_space_ranges: array with 0s for obstacles and non-zero for free space
            
        Returns:
            start_idx: start index of max gap
            end_idx: end index of max gap
        """
        # TODO: Implement max gap finding
        return 0, 0
    
    def find_best_point(self, start_i, end_i, ranges):
        """
        Start_i & end_i are start and end indices of max-gap range
        Return index of best point in ranges
        Naive: Choose the furthest point within ranges and go there
        
        Args:
            start_i: start index of gap
            end_i: end index of gap
            ranges: array of LiDAR ranges
            
        Returns:
            best_idx: index of best point in gap
        """
        # TODO: Implement best point selection
        return 0
    
    def lidar_callback(self, data):
        """ 
        Process each LiDAR scan as per the Follow Gap algorithm & 
        publish an AckermannDriveStamped Message
        
        Args:
            data: LaserScan message
        """
        ranges = np.array(data.ranges)
        
        # Step 1: Preprocess
        proc_ranges = self.preprocess_lidar(ranges)
        
        # TODO: Find closest point
        
        # TODO: Eliminate points inside bubble
        
        # TODO: Find max length gap
        
        # TODO: Find the best point in the gap
        
        # TODO: Publish Drive message


def main(args=None):
    rclpy.init(args=args)
    print("Follow the Gap Initialized")
    reactive_node = ReactiveFollowGap()
    try:
        rclpy.spin(reactive_node)
    except KeyboardInterrupt:
        pass
    finally:
        reactive_node.destroy_node()
        if rclpy.ok():
           rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test basic structure:**
```bash
cd ~/ros2_ws
colcon build --packages-select follow_the_gap
source install/setup.bash
ros2 run follow_the_gap reactive_node
```

Press Ctrl+C to stop. You should see initialization messages.

---

### Step 3.2: Implement preprocess_lidar()

```python
def preprocess_lidar(self, ranges):
    """ 
    Preprocess the LiDAR scan array.
    1. Apply mean filter over a window
    2. Clip values to max_lidar_range
    3. Handle inf and nan values
    
    Args:
        ranges: array of LiDAR ranges
        
    Returns:
        proc_ranges: processed array of ranges
    """
    # Convert to numpy array
    proc_ranges = np.array(ranges)
    
    # Replace inf and nan with max_lidar_range
    proc_ranges = np.nan_to_num(proc_ranges, 
                                 nan=self.max_lidar_range, 
                                 posinf=self.max_lidar_range, 
                                 neginf=0.0)
    
    # Clip to max range - anything beyond max_lidar_range is set to max_lidar_range
    proc_ranges = np.clip(proc_ranges, 0, self.max_lidar_range)
    
    # Apply mean filter (moving average) for smoothing
    # Use convolution for efficient averaging
    if self.preprocess_conv_size > 1:
        kernel = np.ones(self.preprocess_conv_size) / self.preprocess_conv_size
        proc_ranges = np.convolve(proc_ranges, kernel, mode='same')
    
    return proc_ranges
```

**Testing Point 1: Test Preprocessing**

Add temporary test code in `lidar_callback`:

```python
def lidar_callback(self, data):
    ranges = np.array(data.ranges)
    proc_ranges = self.preprocess_lidar(ranges)
    
    # Debug: Print comparison
    self.get_logger().info(
        f'Raw range [0]: {ranges[0]:.2f} | Processed: {proc_ranges[0]:.2f}',
        throttle_duration_sec=2.0)
```

Build and test:
```bash
cd ~/ros2_ws
colcon build --packages-select follow_the_gap
source install/setup.bash

# Terminal 1: Launch simulator (Make sure to have source the simulator first)
ros2 launch f1tenth_gym_ros gym_bridge_launch.py

# Terminal 2: Run node
ros2 run follow_the_gap reactive_node
```

You should see preprocessed ranges being printed.

---

### Step 3.3: Implement Find Closest Point and Safety Bubble

Update `lidar_callback`:

```python
def lidar_callback(self, data):
    """ 
    Process each LiDAR scan as per the Follow Gap algorithm
    """
    ranges = np.array(data.ranges)
    
    # Step 1: Preprocess the LiDAR scan
    proc_ranges = self.preprocess_lidar(ranges)
    
    # Step 2: Find closest point to LiDAR
    # Ignore zero values (they represent obstacles from bubble or invalid)
    # Create a copy to modify
    proc_ranges_copy = proc_ranges.copy()
    
    # Find the closest point (minimum non-zero distance)
    nonzero_ranges = proc_ranges_copy[proc_ranges_copy > 0]
    if len(nonzero_ranges) == 0:
        # No valid ranges, stop the car
        self.get_logger().warn('No valid LiDAR ranges detected!')
        self.publish_drive(0.0, 0.0)
        return
    
    closest_distance = np.min(nonzero_ranges)
    closest_idx = np.argmin(proc_ranges_copy)
    
    # Step 3: Eliminate all points inside 'bubble' (set them to zero)
    # Calculate bubble indices based on closest point
    bubble_indices = self.get_bubble_indices(data, closest_idx, closest_distance)
    proc_ranges_copy[bubble_indices] = 0.0
    
    # Debug logging
    self.get_logger().debug(
        f'Closest point: idx={closest_idx}, dist={closest_distance:.2f}m, '
        f'bubble_size={len(bubble_indices)}',
        throttle_duration_sec=1.0)
    
    # TODO: Continue with finding max gap...
```

Add helper method for safety bubble:

```python
def get_bubble_indices(self, data, closest_idx, closest_distance):
    """
    Get indices of all points within the safety bubble around closest point.
    
    Args:
        data: LaserScan message (for angle info)
        closest_idx: index of closest point
        closest_distance: distance to closest point
        
    Returns:
        bubble_indices: array of indices within bubble
    """
    # Calculate the angle subtended by the bubble
    # Using small angle approximation: angle ≈ bubble_radius / distance
    if closest_distance == 0:
        closest_distance = 0.1  # Avoid division by zero
    
    bubble_angle = self.bubble_radius / closest_distance
    
    # Convert angle to number of indices
    angle_increment = data.angle_increment
    bubble_idx_range = int(np.ceil(bubble_angle / angle_increment))
    
    # Get start and end indices of bubble
    start_idx = max(0, closest_idx - bubble_idx_range)
    end_idx = min(len(data.ranges) - 1, closest_idx + bubble_idx_range)
    
    # Return all indices in bubble
    bubble_indices = np.arange(start_idx, end_idx + 1)
    
    return bubble_indices
```

**Testing Point 2: Verify Safety Bubble**

Add visualization:
```python
# After creating bubble
self.get_logger().info(
    f'Safety bubble: {len(bubble_indices)} points cleared around index {closest_idx}',
    throttle_duration_sec=1.0)
```

---

### Step 3.4: Implement find_max_gap()

```python
def find_max_gap(self, free_space_ranges):
    """ 
    Return the start index & end index of the max gap in free_space_ranges.
    A gap is a continuous sequence of non-zero values.
    
    Args:
        free_space_ranges: array with 0s for obstacles and non-zero for free space
        
    Returns:
        start_idx: start index of max gap
        end_idx: end index of max gap (inclusive)
    """
    # Find where free space begins and ends
    # non-zero values represent free space
    mask = free_space_ranges > 0
    
    # Find the boundaries of all gaps
    # np.diff finds changes (0->1 or 1->0)
    diff = np.diff(np.concatenate(([0], mask.astype(int), [0])))
    
    # starts: where diff changes from 0 to 1 (gap begins)
    # ends: where diff changes from 1 to 0 (gap ends)
    starts = np.where(diff == 1)[0]
    ends = np.where(diff == -1)[0]
    
    # If no gaps found, return invalid indices
    if len(starts) == 0:
        self.get_logger().warn('No gaps found in LiDAR data!')
        return 0, 0
    
    # Calculate gap lengths
    gap_lengths = ends - starts
    
    # Find the maximum gap
    max_gap_idx = np.argmax(gap_lengths)
    
    start_idx = starts[max_gap_idx]
    end_idx = ends[max_gap_idx] - 1  # -1 because ends is exclusive
    
    return start_idx, end_idx
```

**Testing Point 3: Test Gap Finding**

Add to `lidar_callback` after bubble creation:

```python
# Find max length gap
start_i, end_i = self.find_max_gap(proc_ranges_copy)

# Debug
gap_length = end_i - start_i + 1
self.get_logger().info(
    f'Max gap: start={start_i}, end={end_i}, length={gap_length}',
    throttle_duration_sec=1.0)
```

---

### Step 3.5: Implement find_best_point()

```python
def find_best_point(self, start_i, end_i, ranges):
    """
    Find the best point in the gap to navigate toward.
    
    Strategies:
    1. Naive: furthest point in gap
    2. Centered: middle of gap
    3. Weighted average of furthest and centered
    
    Args:
        start_i: start index of gap
        end_i: end index of gap
        ranges: array of LiDAR ranges
        
    Returns:
        best_idx: index of best point in gap
    """
    # Handle edge case
    if start_i >= end_i or end_i >= len(ranges):
        return len(ranges) // 2  # Default to center
    
    # Extract gap ranges
    gap_ranges = ranges[start_i:end_i+1]
    
    # Strategy 1: Furthest point (naive)
    furthest_idx_in_gap = np.argmax(gap_ranges)
    furthest_idx = start_i + furthest_idx_in_gap
    
    # Strategy 2: Centered point
    centered_idx = (start_i + end_i) // 2
    
    # Strategy 3: Weighted average (80% furthest, 20% centered)
    # This balances safety (center) with progress (furthest)
    best_idx = int(0.8 * furthest_idx + 0.2 * centered_idx)
    
    # Alternative: just use furthest
    # best_idx = furthest_idx
    
    # Alternative: just use centered
    # best_idx = centered_idx
    
    return best_idx
```

---

### Step 3.6: Implement Actuation

Add helper method to publish drive commands:

```python
def publish_drive(self, steering_angle, speed):
    """
    Publish AckermannDriveStamped message.
    
    Args:
        steering_angle: desired steering angle (radians)
        speed: desired speed (m/s)
    """
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = self.get_clock().now().to_msg()
    drive_msg.header.frame_id = 'base_link'
    drive_msg.drive.steering_angle = float(steering_angle)
    drive_msg.drive.speed = float(speed)
    
    self.drive_pub.publish(drive_msg)
```

Complete the `lidar_callback`:

```python
def lidar_callback(self, data):
    """ 
    Process each LiDAR scan as per the Follow Gap algorithm
    """
    ranges = np.array(data.ranges)
    
    # Step 1: Preprocess
    proc_ranges = self.preprocess_lidar(ranges)
    
    # Step 2: Find closest point
    proc_ranges_copy = proc_ranges.copy()
    nonzero_ranges = proc_ranges_copy[proc_ranges_copy > 0]
    
    if len(nonzero_ranges) == 0:
        self.get_logger().warn('No valid LiDAR ranges!')
        self.publish_drive(0.0, 0.0)
        return
    
    closest_distance = np.min(nonzero_ranges)
    closest_idx = np.argmin(proc_ranges_copy)
    
    # Step 3: Eliminate points inside bubble (safety bubble)
    bubble_indices = self.get_bubble_indices(data, closest_idx, closest_distance)
    proc_ranges_copy[bubble_indices] = 0.0
    
    # Step 4: Find max length gap
    start_i, end_i = self.find_max_gap(proc_ranges_copy)
    
    # Check if valid gap was found
    if start_i == 0 and end_i == 0:
        self.get_logger().warn('No valid gap found!')
        self.publish_drive(0.0, 0.5)  # Slow down
        return
    
    # Step 5: Find the best point in the gap
    best_idx = self.find_best_point(start_i, end_i, proc_ranges_copy)
    
    # Step 6: Calculate steering angle to best point
    # Convert index to angle
    angle_to_best = data.angle_min + best_idx * data.angle_increment
    
    # Apply steering gain
    steering_angle = self.steering_gain * angle_to_best
    
    # Clamp steering angle to reasonable limits
    max_steering = np.deg2rad(30.0)  # 30 degrees max
    steering_angle = np.clip(steering_angle, -max_steering, max_steering)
    
    # Step 7: Calculate speed based on steering angle and gap size
    # More steering = slower speed
    # Larger gap = faster speed
    gap_size = end_i - start_i
    steering_magnitude = abs(steering_angle)
    
    # Speed calculation
    if steering_magnitude < np.deg2rad(10):
        # Small steering -> high speed
        speed = self.speed_max
    elif steering_magnitude < np.deg2rad(20):
        # Medium steering -> medium speed
        speed = (self.speed_min + self.speed_max) / 2
    else:
        # Large steering -> low speed
        speed = self.speed_min
    
    # Also consider gap size
    # Smaller gap -> slower speed
    min_safe_gap = 50  # minimum indices for full speed
    if gap_size < min_safe_gap:
        speed = speed * (gap_size / min_safe_gap)
        speed = max(speed, self.speed_min)  # Don't go below minimum
    
    # Publish drive command
    self.publish_drive(steering_angle, speed)
    
    # Debug logging
    self.get_logger().info(
        f'Gap: [{start_i}:{end_i}] | Best: {best_idx} | '
        f'Angle: {np.rad2deg(steering_angle):.1f}° | Speed: {speed:.2f} m/s',
        throttle_duration_sec=0.5)
```

---
