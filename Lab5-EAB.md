# Lab 5: Automatic Emergency Braking (AEB) with iTTC

## Lab Overview
In this lab, you will develop a safety node that prevents the race car from colliding with obstacles by implementing Automatic Emergency Braking (AEB) using Instantaneous Time to Collision (iTTC). This is a critical safety feature for autonomous racing vehicles.

---

## Learning Objectives
By the end of this lab, you will be able to:
1. Understand and implement Time to Collision (TTC) calculations
2. Work with LaserScan messages for obstacle detection
3. Process Odometry messages to extract vehicle velocity
4. Implement emergency braking logic using AckermannDriveStamped
5. Handle edge cases (inf, nan) in sensor data
6. Tune safety thresholds for reliable collision avoidance
7. Test safety systems in a simulated environment

---

## Prerequisites

Ensure ROS2 Humble is installed and sourced:

```bash
source /opt/ros/humble/setup.bash
```

---

## Part 1: Understanding the Theory

### What is Time to Collision (TTC)?

**Time to Collision (TTC)** is the time it would take for a vehicle to collide with an obstacle if it maintains its current heading and velocity.

**Instantaneous Time to Collision (iTTC)** is an approximation calculated from:
- Current range measurements (from LiDAR)
- Current velocity (from odometry)

### The iTTC Formula

$$iTTC = \frac{r}{\{-\dot{r}\}_+}$$

Where:
- $r$ = instantaneous range measurement (distance to obstacle)
- $\dot{r}$ = range rate (rate of change of distance)
- $\{x\}_+ = \max(x, 0)$ (ensures we only consider approaching obstacles)

### Understanding Range Rate

**Range rate** $\dot{r}$ tells us how fast the distance to an obstacle is changing:

- **Negative range rate**: Distance is decreasing (approaching obstacle) → collision risk
- **Positive range rate**: Distance is increasing (moving away) → no collision risk

**Calculation Method 1 - Using Vehicle Velocity:**

For each LiDAR beam at angle $\theta_i$:

$$\dot{r}_i = -v_x \cos(\theta_i)$$

Where:
- $v_x$ = vehicle's longitudinal velocity
- $\theta_i$ = angle of the LiDAR beam
- Negative sign accounts for: approaching obstacle = shrinking range = negative rate

**Calculation Method 2 - Using Temporal Difference:**

$$\dot{r}_i = \frac{r_{current} - r_{previous}}{\Delta t}$$

**For this lab, we'll use Method 1** (velocity projection method).

### Example Calculation

```
Scenario: Car traveling at 2 m/s toward a wall 10m ahead

Beam directly ahead (θ = 0°):
- r = 10.0 m
- v_x = 2.0 m/s
- ṙ = -2.0 * cos(0°) = -2.0 m/s
- iTTC = 10.0 / max(-(-2.0), 0) = 10.0 / 2.0 = 5.0 seconds

Beam at 90° to the side (θ = 90°):
- r = 5.0 m
- ṙ = -2.0 * cos(90°) = 0.0 m/s
- iTTC = 5.0 / max(0.0, 0) → infinity (no collision risk)
```

### Understanding LaserScan Message

```python
LaserScan:
  header:
    stamp: Time of measurement
    frame_id: "ego_racecar/laser"
  
  angle_min: -2.35619  # Start angle (radians)
  angle_max: 2.35619   # End angle (radians)
  angle_increment: 0.00436  # Angular resolution
  
  range_min: 0.0       # Minimum valid range
  range_max: 30.0      # Maximum valid range
  
  ranges: [5.2, 5.1, 5.0, ...]  # Array of range measurements
```

**Key points:**
- `ranges` array is radially ordered from `angle_min` to `angle_max`
- Each element corresponds to a specific angle (determined by using `angle_increment`)
- Invalid measurements may be `inf` or `nan`

---

## Part 2: Setting Up the Workspace

### Step 2.1: Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create safety_node --build-type ament_python \
  --dependencies rclpy sensor_msgs nav_msgs ackermann_msgs std_msgs
```

### Step 2.2: Create Directory Structure

```bash
cd safety_node
mkdir -p launch config
```

**Resulting structure:**
```
safety_node/
├── safety_node/
│   ├── __init__.py
│   └── safety_node.py          # Main node (we'll create this)
├── launch/
│   └── safety_node.launch.py   # Launch file (we'll create this)
├── config/
│   └── safety_params.yaml      # Parameters (we'll create this)
├── package.xml
├── setup.py
└── resource/
```

### Step 2.3: Check package.xml

Ensure your `package.xml` file has the necessary dependencies

**File:** `~/ros2_ws/src/safety_node/package.xml`

```xml
  ...
  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>ackermann_msgs</depend>
  <depend>std_msgs</depend>
  ...
```

### Step 2.4: Update setup.py

**File:** `~/ros2_ws/src/safety_node/setup.py`

```python
from setuptools import setup
import os # Added
from glob import glob # Added

package_name = 'safety_node'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Name',
    maintainer_email='student@example.com',
    description='AEB safety node using iTTC',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'safety_node = safety_node.safety_node:main',
        ],
    },
)
```

---

## Part 3: Implementing the Safety Node - Step by Step

### Step 3.1: Create Basic Node Structure

**File:** `~/ros2_ws/src/safety_node/safety_node/safety_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        
        # Declare parameters
        self.declare_parameter('ttc_threshold', 0.5)  # seconds
        self.declare_parameter('speed_threshold', 0.1)  # m/s minimum speed to check
        
        # Get parameters
        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        self.speed_threshold = self.get_parameter('speed_threshold').value
        
        # Initialize variables
        self.speed = 0.0
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        # Create publisher for brake commands
        self.brake_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        self.get_logger().info('Safety Node initialized')
        self.get_logger().info(f'TTC Threshold: {self.ttc_threshold} seconds')
        self.get_logger().info(f'Speed Threshold: {self.speed_threshold} m/s')

    def odom_callback(self, odom_msg):
        """
        Update current speed from odometry
        """
        # Extract longitudinal velocity (x component in vehicle frame)
        self.speed = odom_msg.twist.twist.linear.x
        
    def scan_callback(self, scan_msg):
        """
        Process laser scan and trigger emergency brake if needed
        """
        # TODO: Implement iTTC calculation and braking logic
        pass

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        if rclpy.ok():
           rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Test this basic structure:**
```bash
cd ~/ros2_ws
colcon build --packages-select safety_node --symlink-install
source install/setup.bash
ros2 run safety_node safety_node
```

Press Ctrl+C to stop. You should see the initialization messages.

---

### Step 3.2: Implement iTTC Calculation

Now let's fill in the `scan_callback` with iTTC calculation:

```python
def scan_callback(self, scan_msg):
    """
    Process laser scan and trigger emergency brake if needed
    """
    # Skip processing if car is nearly stationary
    if abs(self.speed) < self.speed_threshold:
        return
    
    # Extract scan data
    ranges = np.array(scan_msg.ranges)
    
    # Calculate angle for each beam
    # angle_i = angle_min + i * angle_increment
    num_beams = len(ranges)
    angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment
    
    # Calculate range rate for each beam
    # ṙ = -v_x * cos(θ)
    # Negative sign because approaching obstacle means decreasing range
    range_rates = -self.speed * np.cos(angles)
    
    # Calculate iTTC for each beam
    # iTTC = r / max(-ṙ, 0)
    # We need -ṙ because we want positive values for approaching obstacles
    # The max(..., 0) ensures we only consider negative range rates (approaching)
    
    # Initialize iTTC array with infinity
    ittc = np.full(num_beams, np.inf)
    
    # Only calculate iTTC where range rate is negative (approaching obstacle)
    approaching = range_rates < 0
    
    # Calculate iTTC for approaching obstacles
    # iTTC = range / abs(range_rate)
    ittc[approaching] = ranges[approaching] / np.abs(range_rates[approaching])
    
    # Handle invalid measurements (inf, nan, out of range)
    # Replace inf and nan with a large number to avoid triggering brake
    ittc = np.nan_to_num(ittc, nan=np.inf, posinf=np.inf, neginf=np.inf)
    
    # Also filter out measurements outside valid range
    invalid_ranges = (ranges < scan_msg.range_min) | (ranges > scan_msg.range_max)
    ittc[invalid_ranges] = np.inf
    
    # Find minimum iTTC
    min_ittc = np.min(ittc)
    
    # Log for debugging
    if min_ittc < self.ttc_threshold:
        self.get_logger().warn(
            f'Collision imminent! Min iTTC: {min_ittc:.3f}s (threshold: {self.ttc_threshold}s)',
            throttle_duration_sec=1.0)  # Throttle to avoid spam
    
    # Trigger emergency brake if iTTC is below threshold
    if min_ittc < self.ttc_threshold:
        self.publish_brake()
        
def publish_brake(self):
    """
    Publish emergency brake command
    """
    brake_msg = AckermannDriveStamped()
    brake_msg.header.stamp = self.get_clock().now().to_msg()
    brake_msg.header.frame_id = 'base_link'
    
    # Set speed to 0 to brake
    brake_msg.drive.speed = 0.0
    
    self.brake_pub.publish(brake_msg)
    self.get_logger().info('EMERGENCY BRAKE ACTIVATED!', throttle_duration_sec=1.0)
```

**Complete safety_node.py at this stage:**

**File:** `~/ros2_ws/src/safety_node/safety_node/safety_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class SafetyNode(Node):
    """
    The class that handles emergency braking using iTTC.
    """
    def __init__(self):
        super().__init__('safety_node')
        
        # Declare parameters
        self.declare_parameter('ttc_threshold', 0.5)
        self.declare_parameter('speed_threshold', 0.1)
        
        # Get parameters
        self.ttc_threshold = self.get_parameter('ttc_threshold').value
        self.speed_threshold = self.get_parameter('speed_threshold').value
        
        # Initialize variables
        self.speed = 0.0
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ego_racecar/odom',
            self.odom_callback,
            10)
        
        # Create publisher for brake commands
        self.brake_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Safety Node initialized')
        self.get_logger().info(f'TTC Threshold: {self.ttc_threshold} seconds')
        self.get_logger().info(f'Speed Threshold: {self.speed_threshold} m/s')
        self.get_logger().info('='*50)

    def odom_callback(self, odom_msg):
        """
        Update current speed from odometry.
        The x component of linear velocity is the longitudinal speed.
        """
        self.speed = odom_msg.twist.twist.linear.x
        
    def scan_callback(self, scan_msg):
        """
        Process laser scan and trigger emergency brake if needed.
        Calculates iTTC for all beams and brakes if minimum is below threshold.
        """
        # Skip processing if car is nearly stationary
        if abs(self.speed) < self.speed_threshold:
            return
        
        # Extract scan data
        ranges = np.array(scan_msg.ranges)
        
        # Calculate angle for each beam
        num_beams = len(ranges)
        angles = scan_msg.angle_min + np.arange(num_beams) * scan_msg.angle_increment
        
        # Calculate range rate for each beam
        # ṙ = -v_x * cos(θ)
        range_rates = -self.speed * np.cos(angles)
        
        # Initialize iTTC array with infinity
        ittc = np.full(num_beams, np.inf)
        
        # Only calculate iTTC where range rate is negative (approaching)
        approaching = range_rates < 0
        
        # Calculate iTTC: iTTC = r / |ṙ| for approaching obstacles
        ittc[approaching] = ranges[approaching] / np.abs(range_rates[approaching])
        
        # Handle invalid measurements
        ittc = np.nan_to_num(ittc, nan=np.inf, posinf=np.inf, neginf=np.inf)
        
        # Filter out measurements outside valid range
        invalid_ranges = (ranges < scan_msg.range_min) | (ranges > scan_msg.range_max)
        ittc[invalid_ranges] = np.inf
        
        # Find minimum iTTC
        min_ittc = np.min(ittc)
        
        # Debug logging
        if min_ittc < self.ttc_threshold:
            min_idx = np.argmin(ittc)
            min_angle = angles[min_idx]
            min_range = ranges[min_idx]
            
            self.get_logger().warn(
                f'Collision Warning! iTTC: {min_ittc:.3f}s | '
                f'Range: {min_range:.2f}m | Angle: {np.degrees(min_angle):.1f}°',
                throttle_duration_sec=0.5)
        
        # Trigger emergency brake if iTTC is below threshold
        if min_ittc < self.ttc_threshold:
            self.publish_brake()
            
    def publish_brake(self):
        """
        Publish emergency brake command (speed = 0).
        """
        brake_msg = AckermannDriveStamped()
        brake_msg.header.stamp = self.get_clock().now().to_msg()
        brake_msg.header.frame_id = 'base_link'
        brake_msg.drive.speed = 0.0
        
        self.brake_pub.publish(brake_msg)
        self.get_logger().info(
            '🛑 EMERGENCY BRAKE ACTIVATED! 🛑',
            throttle_duration_sec=1.0)

def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        if rclpy.ok():
           rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### Step 3.3: Create Parameter Configuration File

**File:** `~/ros2_ws/src/safety_node/config/safety_params.yaml`

```yaml
# Safety Node Parameters

safety_node:
  ros__parameters:
    # Time to Collision threshold (seconds)
    # Lower values = more aggressive braking
    # Higher values = more conservative, may brake unnecessarily
    ttc_threshold: 0.5
    
    # Minimum speed to activate safety checks (m/s)
    # Below this speed, AEB is disabled to avoid false positives when stationary
    speed_threshold: 0.1
```

---

### Step 3.4: Create Launch File

**File:** `~/ros2_ws/src/safety_node/launch/safety_node.launch.py`

```python
#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch the safety node with parameters
    """
    
    # Get package directory
    pkg_dir = get_package_share_directory('safety_node')
    
    # Path to parameter file
    params_file = os.path.join(pkg_dir, 'config', 'safety_params.yaml')
    
    # Declare launch arguments
    ttc_threshold_arg = DeclareLaunchArgument(
        'ttc_threshold',
        default_value='0.5',
        description='Time to Collision threshold in seconds'
    )
    
    speed_threshold_arg = DeclareLaunchArgument(
        'speed_threshold',
        default_value='0.1',
        description='Minimum speed to activate AEB (m/s)'
    )
    
    # Safety node
    safety_node = Node(
        package='safety_node',
        executable='safety_node',
        name='safety_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            params_file,
            {
                'ttc_threshold': LaunchConfiguration('ttc_threshold'),
                'speed_threshold': LaunchConfiguration('speed_threshold'),
            }
        ]
    )
    
    return LaunchDescription([
        ttc_threshold_arg,
        speed_threshold_arg,
        safety_node,
    ])
```

---

### Step 3.5: Build and Test

```bash
cd ~/ros2_ws
colcon build --packages-select safety_node
source install/setup.bash
```

**Test the node:**
```bash
ros2 launch safety_node safety_node.launch.py
```

---

## Part 4: Testing Procedure

1. **Start all nodes** (simulator, safety, teleop, anything else that is needed)

2. **In simulator, use `2D Pose Estimate tool`** to position and orient the car. Click somewhere on the map, and drag for direction.

3. **Drive toward a wall using keyboard:**

4. **Observe safety node behavior:**
   - Watch terminal for iTTC warnings
   - Car should automatically brake before hitting wall
   - Test at different speeds

5. **Experiment with parameters:**
   ```bash
   # More aggressive braking
   ros2 launch safety_node safety_node.launch.py ttc_threshold:=1.0
   
   # Less aggressive
   ros2 launch safety_node safety_node.launch.py ttc_threshold:=0.3
   ```
> ***Note that the simulation is in development and does not always work as expected***

---

## Part 5: Advanced Features and Tuning (20 minutes)

### Step 5.1: Add Safety Regions

Modify to have different thresholds for different angular regions:

```python
def scan_callback(self, scan_msg):
    # ... existing code ...
    
    # Define safety regions (in radians)
    front_region = (-0.5, 0.5)  # ±30 degrees
    side_region_threshold = 0.3  # More permissive for sides
    
    # Apply region-specific thresholds
    front_mask = (angles >= front_region[0]) & (angles <= front_region[1])
    
    # Check front region with stricter threshold
    if np.any(ittc[front_mask] < self.ttc_threshold):
        self.publish_brake()
        return
    
    # Check sides with more permissive threshold
    if np.min(ittc) < side_region_threshold:
        self.publish_brake()
```

### Step 5.2: Add Filtering for False Positives

```python
def __init__(self):
    # ... existing code ...
    self.brake_count = 0
    self.brake_threshold_count = 3  # Must trigger 3 times in a row
    
def scan_callback(self, scan_msg):
    # ... existing iTTC calculation ...
    
    if min_ittc < self.ttc_threshold:
        self.brake_count += 1
    else:
        self.brake_count = 0
    
    # Only brake if consistently detecting collision
    if self.brake_count >= self.brake_threshold_count:
        self.publish_brake()
```

---

## Lab Summary

### What We Learned:

1. **iTTC Theory:**
   - Time to Collision concept
   - Range rate calculation
   - Projection of velocity onto scan beams

2. **ROS2 Message Types:**
   - `LaserScan` - LiDAR data structure
   - `Odometry` - Vehicle state information
   - `AckermannDriveStamped` - Vehicle control commands

3. **Safety System Design:**
   - Real-time obstacle detection
   - Emergency braking logic
   - Handling edge cases (inf, nan)

4. **Parameter Tuning:**
   - TTC threshold selection
   - Speed threshold configuration
   - Region-specific safety zones

5. **Testing and Validation:**
   - Simulator-based testing
   - Visualization with RViz
   - Debug monitoring

### Key Takeaways for Autonomous Racing:

- **Safety is paramount** - AEB is a last-resort safety net
- **Tuning is critical** - Balance between safety and performance
- **Real-time constraints** - Must process scans quickly
- **Robust handling** - Must deal with sensor noise and errors
- **Layered safety** - AEB complements higher-level planning

---

# Deliverable

## Automatic Emergency Braking (AEB) Experimentation and Analysis

### Objective

Demonstrate that your Safety Node using Instantaneous Time to Collision (iTTC) can computes iTTC from /scan and /ego_racecar/odom, and stops the vehicle before collision.

## Assignment Overview

You will:

1. Run the simulator with keyboard teleop and your safety node.
2. Drive the vehicle toward a wall.
3. Increase speed gradually.
4. Observe when braking occurs.

**Submit a screen recorded video showing the above and write a few paragraphs on what you observed as you were developing and experimenting.**


