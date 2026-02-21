# Lab 6: Wall Following with PID Control

## Lab Overview
In this lab, you will implement a wall-following algorithm using PID (Proportional-Integral-Derivative) control. The car will autonomously navigate by maintaining a constant distance from a wall

**Platform:** Ubuntu 22.04 (VMware), ROS2 Humble  
**Simulator:** F1TENTH GYM ROS Simulator

---

## Learning Objectives
By the end of this lab, you will be able to:
1. Understand and implement PID controllers in the time domain
2. Calculate distance and angle to walls using LiDAR data
3. Project future vehicle position for predictive control
4. Tune PID gains for stable and responsive control
5. Implement velocity control based on steering angle
6. Test and validate autonomous driving in simulation

---

## Prerequisites

Ensure ROS2 Humble and F1TENTH simulator are installed:

```bash
source /opt/ros/humble/setup.bash
# Verify simulator is installed
ros2 pkg list | grep f1tenth
```

---

## Part 1: Understanding the Theory

### What is PID Control?

**PID (Proportional-Integral-Derivative)** controller maintains a system parameter around a desired setpoint by continuously calculating and correcting for error.

**PID Equation:**

$$u(t) = K_p e(t) + K_i \int_0^t e(t') dt' + K_d \frac{d}{dt}(e(t))$$

Where:
- $u(t)$ = control output (steering angle in our case)
- $e(t)$ = error (desired distance - actual distance)
- $K_p$ = proportional gain
- $K_i$ = integral gain  
- $K_d$ = derivative gain

**Component Roles:**
- **Proportional (P):** Responds to current error magnitude
  - High $K_p$ → fast response, but may overshoot
  - Low $K_p$ → slow response, stable
  
- **Integral (I):** Corrects accumulated past error
  - Eliminates steady-state error
  - Too high → oscillation and instability
  
- **Derivative (D):** Predicts future error based on rate of change
  - Dampens oscillations
  - Improves stability
  - Too high → amplifies noise

### Wall Following Geometry

**Goal:** Maintain constant distance to the right wall while driving forward.

**Given:**
- Two LiDAR beams:
  - Beam `b`: perpendicular to car (90° to right)
  - Beam `a`: at angle `θ` from beam `b` (typically 50-70°)
- Distances: `a` and `b` from laser scan

**Calculate:**

1. **Angle α between car and wall:**
$$\alpha = \tan^{-1}\left(\frac{a\cos(\theta) - b}{a\sin(\theta)}\right)$$

2. **Current distance to wall $(D_t)$:**
$$D_t = b\cos(\alpha)$$

3. **Future distance with lookahead $(D_{t+1})$:**
$$D_{t+1} = D_t + L\sin(\alpha)$$

Where $L$ is the lookahead distance (typically 0.5-2.0 meters).

**Why lookahead?** At high speeds, the car has delayed response. Looking ahead allows predictive control to prevent crashing.

### Example Calculation

```
Scenario: Car parallel to wall, 1.0m away

θ = 60° (angle between beams a and b)
b = 1.0m (perpendicular distance)
a = 1.0m (angled beam)
L = 1.0m (lookahead)

Step 1: Calculate α
α = atan2((1.0 * cos(60°) - 1.0), (1.0 * sin(60°)))
α = atan2((0.5 - 1.0), 0.866)
α = atan2(-0.5, 0.866) ≈ -0.524 rad ≈ -30°

Step 2: Current distance
D_t = 1.0 * cos(-30°) ≈ 0.866m

Step 3: Future distance  
D_{t+1} = 0.866 + 1.0 * sin(-30°) ≈ 0.866 - 0.5 = 0.366m

Step 4: Error (if desired distance = 1.0m)
e(t) = 1.0 - 0.366 = 0.634m (too close!)

Step 5: PID output → steer left to move away from wall
```

---

## Part 2: Setting Up the Workspace

### Step 2.1: Create Package

```bash
cd ~/ros2_ws/src
ros2 pkg create wall_follow --build-type ament_python \
  --dependencies rclpy sensor_msgs ackermann_msgs std_msgs
```

### Step 2.2: Create Directory Structure

```bash
cd wall_follow
mkdir -p launch config
```

**Resulting structure:**
```
wall_follow/
├── wall_follow/
│   ├── __init__.py
│   └── wall_follow_node.py     # Main implementation
├── launch/
│   └── wall_follow.launch.py   # Launch file
├── config/
│   └── wall_follow_params.yaml # PID parameters
├── package.xml
├── setup.py
└── resource/
```

### Step 2.3: Ensure `package.xml` has the right dependencies

**File:** `~/ros2_ws/src/wall_follow/package.xml`

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>wall_follow</name>
  <version>1.0.0</version>
  <description>Wall following using PID control for F1TENTH</description>
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

### Step 2.4: Update setup.py to include entry point for your node and your data file for launch and config files if applicable

**File:** `~/ros2_ws/src/wall_follow/setup.py`

```python
from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'wall_follow'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
    maintainer='israel',
    maintainer_email='israel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_follow_node = wall_follow.wall_follow_node:main',
        ],
    },
)
```

---

## Part 3: Implementing Wall Following - Step by Step

### Step 3.1: Create Basic Node Structure

**File:** `~/ros2_ws/src/wall_follow/wall_follow/wall_follow_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


class WallFollow(Node):
    """ 
    Implement Wall Following on the car using PID control
    """
    def __init__(self):
        super().__init__('wall_follow_node')
        
        # Declare parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.declare_parameter('desired_distance', 1.0)  # meters from wall
        self.declare_parameter('lookahead_distance', 1.0)  # meters
        self.declare_parameter('theta_deg', 50.0)  # angle for beam 'a' in degrees
        
        # Get parameters
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.desired_distance = self.get_parameter('desired_distance').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.theta_deg = self.get_parameter('theta_deg').value
        
        # Convert theta to radians
        self.theta = np.deg2rad(self.theta_deg)
        
        # PID control variables
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
        # Create subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        # Create publisher
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped,
            '/drive',
            10)
        
        self.get_logger().info('='*50)
        self.get_logger().info('Wall Follow Node Initialized')
        self.get_logger().info(f'PID Gains: Kp={self.kp}, Ki={self.ki}, Kd={self.kd}')
        self.get_logger().info(f'Desired Distance: {self.desired_distance}m')
        self.get_logger().info(f'Lookahead Distance: {self.lookahead_distance}m')
        self.get_logger().info(f'Theta: {self.theta_deg}°')
        self.get_logger().info('='*50)

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle.
        Handles NaNs and Infs.

        Args:
            range_data: LaserScan message with range data
            angle: desired angle in radians (relative to car's x-axis)

        Returns:
            range: range measurement in meters at the given angle
        """
        # Calculate the index for the desired angle
        # angle_index = (angle - angle_min) / angle_increment
        angle_index = int((angle - range_data.angle_min) / range_data.angle_increment)
        
        # Ensure index is within bounds
        angle_index = np.clip(angle_index, 0, len(range_data.ranges) - 1)
        
        # Get range value
        range_val = range_data.ranges[angle_index]
        
        # Handle inf and nan
        if np.isnan(range_val) or np.isinf(range_val):
            # Return a large value or max range
            return range_data.range_max
        
        return range_val

    def get_error(self, range_data, dist):
        """
        Calculates the error to the wall. Follows the wall to the right.

        Args:
            range_data: LaserScan message
            dist: desired distance to the wall

        Returns:
            error: calculated error (desired - actual future distance)
        """
        # TODO: Implement this method
        # For now, return 0
        return 0.0

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        # TODO: Implement PID control
        angle = 0.0
        
        # Create and publish drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = 'base_link'
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and 
        publish the drive message.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        # Calculate error
        error = self.get_error(msg, self.desired_distance)
        
        # Calculate velocity based on error (simple step function)
        velocity = 1.5  # Default speed
        
        # Apply PID control
        self.pid_control(error, velocity)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    try:
        rclpy.spin(wall_follow_node)
    except KeyboardInterrupt:
        pass
    finally:
        wall_follow_node.destroy_node()
        if rclpy.ok():
           rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Test basic structure:**
```bash
cd ~/ros2_ws
colcon build --packages-select wall_follow
source install/setup.bash
ros2 run wall_follow wall_follow_node
```

Press Ctrl+C to stop. You should see initialization messages.

---

### Step 3.2: Implement get_error() Method

Now let's implement the core geometry calculations:

```python
def get_error(self, range_data, dist):
    """
    Calculates the error to the wall. Follows the wall to the right.
    
    Algorithm:
    1. Get ranges at 90° (beam b) and 90° - θ (beam a)
    2. Calculate angle α between car and wall
    3. Calculate current distance D_t
    4. Calculate future distance D_{t+1} with lookahead
    5. Return error: desired_distance - D_{t+1}

    Args:
        range_data: LaserScan message
        dist: desired distance to the wall (meters)

    Returns:
        error: calculated error (desired - actual future distance)
    """
    # Get range at 90 degrees (perpendicular to car, pointing right)
    # In standard LiDAR frame: 0° is forward, positive angles go counter-clockwise
    # So 90° to the right is actually -90° or -π/2
    # But we want to follow the wall to the RIGHT
    # Typically: 0° forward, 90° left (π/2), -90° or 270° right (-π/2)
    
    # For right wall following:
    # b: perpendicular beam to the right (270° or -90° or 3π/2)
    # a: beam at angle theta forward from b
    
    angle_b = -np.pi/2  # 90 degrees to the right (-90° from forward)
    angle_a = angle_b + self.theta  # theta degrees forward from beam b
    
    # Get range measurements
    b = self.get_range(range_data, angle_b)
    a = self.get_range(range_data, angle_a)
    
    # Calculate alpha (angle between car's x-axis and the wall)
    # α = atan2((a*cos(θ) - b), (a*sin(θ)))
    numerator = a * np.cos(self.theta) - b
    denominator = a * np.sin(self.theta)
    
    # Handle edge case where denominator is zero
    if abs(denominator) < 1e-6:
        alpha = 0.0
    else:
        alpha = np.arctan2(numerator, denominator)
    
    # Calculate current distance to wall (D_t)
    D_t = b * np.cos(alpha)
    
    # Calculate projected future distance (D_{t+1})
    D_t_plus_1 = D_t + self.lookahead_distance * np.sin(alpha)
    
    # Calculate error
    error = dist - D_t_plus_1
    
    return error
```

---

### Step 3.3: Implement PID Control

Now implement the PID controller:

```python
def pid_control(self, error, velocity):
    """
    Based on the calculated error, publish vehicle control using PID.
    
    PID Formula: u(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt

    Args:
        error: calculated error (desired - actual distance)
        velocity: desired velocity

    Returns:
        None
    """
    # Get current time
    current_time = self.get_clock().now()
    
    # Calculate dt (time since last update)
    if self.prev_time is None:
        dt = 0.0
        self.prev_time = current_time
    else:
        dt = (current_time - self.prev_time).nanoseconds / 1e9  # Convert to seconds
        self.prev_time = current_time
    
    # Avoid division by zero or very small dt
    if dt < 1e-6:
        dt = 1e-6
    
    # Proportional term
    P = self.kp * error
    
    # Integral term (accumulate error over time)
    self.integral += error * dt
    I = self.ki * self.integral
    
    # Derivative term (rate of change of error)
    derivative = (error - self.prev_error) / dt
    D = self.kd * derivative
    
    # Calculate steering angle
    angle = P + I + D
    
    # Clamp steering angle to reasonable limits (e.g., ±30 degrees)
    max_steering_angle = np.deg2rad(30.0)
    angle = np.clip(angle, -max_steering_angle, max_steering_angle)
    
    # Update previous error
    self.prev_error = error
    
    # Log for debugging (throttled)
    self.get_logger().debug(
        f'Error: {error:.3f}m | P: {P:.3f} | I: {I:.3f} | D: {D:.3f} | '
        f'Angle: {np.rad2deg(angle):.1f}°',
        throttle_duration_sec=0.5)
    
    # Create and publish drive message
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = self.get_clock().now().to_msg()
    drive_msg.header.frame_id = 'base_link'
    drive_msg.drive.steering_angle = float(angle)
    drive_msg.drive.speed = float(velocity)
    
    self.drive_pub.publish(drive_msg)
```

---

### Step 3.4: Implement Velocity Control

Update the `scan_callback` to calculate velocity based on steering angle:

```python
def scan_callback(self, msg):
    """
    Callback function for LaserScan messages. Calculate the error and 
    publish the drive message.

    Args:
        msg: Incoming LaserScan message

    Returns:
        None
    """
    # Calculate error
    error = self.get_error(msg, self.desired_distance)
    
    # Calculate absolute steering angle needed (estimate before PID)
    # For velocity control, we can use the error magnitude
    error_abs = abs(error)
    
    # Velocity control based on required correction
    # Aggressive correction → slow down
    # Small correction → go faster
    if error_abs < 0.2:  # Very small error
        velocity = 2.0
    elif error_abs < 0.5:  # Moderate error
        velocity = 1.5
    elif error_abs < 1.0:  # Large error
        velocity = 1.0
    else:  # Very large error
        velocity = 0.5
    
    # Alternative: velocity based on predicted steering angle
    # We'll implement this after PID is working
    
    # Apply PID control
    self.pid_control(error, velocity)
```

---

### Step 3.5: Complete Implementation with Steering-Based Velocity

Better approach: calculate velocity after getting steering angle:

```python
def pid_control(self, error, velocity):
    """
    Based on the calculated error, publish vehicle control using PID.
    Also adjusts velocity based on steering angle magnitude.

    Args:
        error: calculated error (desired - actual distance)
        velocity: base desired velocity (may be adjusted)

    Returns:
        None
    """
    # Get current time
    current_time = self.get_clock().now()
    
    # Calculate dt
    if self.prev_time is None:
        dt = 0.0
        self.prev_time = current_time
    else:
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time
    
    if dt < 1e-6:
        dt = 1e-6
    
    # PID calculation
    P = self.kp * error
    self.integral += error * dt
    I = self.ki * self.integral
    derivative = (error - self.prev_error) / dt
    D = self.kd * derivative
    
    # Calculate steering angle
    angle = P + I + D
    
    # Clamp steering angle
    max_steering_angle = np.deg2rad(30.0)
    angle = np.clip(angle, -max_steering_angle, max_steering_angle)
    
    # Update previous error
    self.prev_error = error
    
    # Adjust velocity based on steering angle magnitude
    angle_deg = abs(np.rad2deg(angle))
    
    if angle_deg < 10.0:
        # Gentle steering → full speed
        final_velocity = 1.5
    elif angle_deg < 20.0:
        # Moderate steering → reduced speed
        final_velocity = 1.0
    else:
        # Sharp steering → slow speed
        final_velocity = 0.5
    
    # Log for debugging
    self.get_logger().info(
        f'Error: {error:.3f}m | Angle: {angle_deg:.1f}° | Speed: {final_velocity:.1f} m/s',
        throttle_duration_sec=1.0)
    
    # Create and publish drive message
    drive_msg = AckermannDriveStamped()
    drive_msg.header.stamp = self.get_clock().now().to_msg()
    drive_msg.header.frame_id = 'base_link'
    drive_msg.drive.steering_angle = float(angle)
    drive_msg.drive.speed = float(final_velocity)
    
    self.drive_pub.publish(drive_msg)

def scan_callback(self, msg):
    """
    Callback function for LaserScan messages.
    """
    # Calculate error
    error = self.get_error(msg, self.desired_distance)
    
    # Base velocity (will be adjusted in pid_control based on steering)
    base_velocity = 1.5
    
    # Apply PID control
    self.pid_control(error, base_velocity)
```

---

### Step 3.6: Add Anti-Windup for Integral Term

Prevent integral windup (integral term growing unbounded):

```python
def pid_control(self, error, velocity):
    """
    PID control with anti-windup
    """
    # ... existing time and dt calculation ...
    
    # PID calculation
    P = self.kp * error
    
    # Integral with anti-windup
    self.integral += error * dt
    # Clamp integral to prevent windup
    max_integral = 10.0  # Tune this value
    self.integral = np.clip(self.integral, -max_integral, max_integral)
    I = self.ki * self.integral
    
    # ... rest of the method ...
```

---

