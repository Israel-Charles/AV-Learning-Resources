# **Lab 2 - Introduction to ROS 2 and Robotics Middleware**

## Lab Overview

This lab introduces the fundamental concepts of **ROS2**, including **nodes, topics, messages, launchfiles, and the ROS2 workspace structure**. You will implement **publisher/subscriber nodes** in both **Python and C++**, visualize message flow using ROS2 command-line tools, and build a **multi-node communication system**.

*ROS 2 is developed and maintained by Open Robotics and is the industry standard for modern robotic systems.*

---

## **Learning Objectives**

1. Explain the role of ROS 2 as a **robotics middleware**
2. Build and source a **ROS 2 workspace**
3. Create and run **publisher and subscriber nodes** in Python or C++
4. Implement asynchronous communication using **Nodes, Topics, and Messages**.
5. Inspect node and topic communication using **ROS 2 CLI tools**
6. Define and use a **custom message type**
7. Launch a **multi-node system** using a launch file

---

## **Lab Structure Overview**

| Part   | Topic                    | Outcome                     |
| ------ | ------------------------ | --------------------------- |
| Part 0 | Environment & Workspace  | Working ROS 2 workspace     |
| Part 1 | Nodes, Topics, Messages  | Basic pub/sub communication |
| Part 2 | ROS 2 CLI Introspection  | Runtime debugging skills    |
| Part 3 | Custom Messages          | Structured robot data       |
| Part 4 | Multi-node Launch        | Scalable system startup     |

---

## Review ROS2 Fundamentals

### Key Concepts Review

Before starting the exercises, familiarize yourself with these core ROS2 concepts:

- **Nodes**: Independent processes that perform specific computations
- **Topics**: Named buses for asynchronous message passing between nodes
- **Messages**: Typed data structures exchanged over topics
- **Publishers**: Nodes that send messages to topics
- **Subscribers**: Nodes that receive messages from topics
- **Workspace**: Directory structure for organizing ROS2 packages
- **Packages**: Organizational units containing nodes, libraries, and configuration files

### ROS2 Workspace Structure

```
ros2_ws/
├── src/              # Source code for packages
├── build/            # Intermediate build files
├── install/          # Compiled binaries and libraries
└── log/              # Build and runtime logs
```

---

# **Part 0 - Environment Setup & Workspace**

## Objective

**Build and run a ROS 2 workspace**.

## Tasks

1. Source your ROS 2 installation if not automatically done already:

   ```bash
   source /opt/ros/humble/setup.bash
   ```
2. Create and build a workspace:

   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## Checkpoint

* `ros2 pkg list` runs without errors

---

# **Part 1 - Publisher & Subscriber Nodes**

## Objective

Understand **nodes, topics, and messages** through hands-on implementation.

## Tasks

* **Track A (Python)** – `rclpy`
* **Track B (C++)** – `rclcpp`

## **Track A - Python Publisher-Subscriber Implementation**

### **Step 1: Create Python Package**

1. **In the ROS2 workspace you created, Navigate to the `src/` directory:**
   ```bash
   cd ~/ros2_ws/src
   ```

2. **Create a Python package named `my_python_pkg` with dependencies on `rclpy` and `std_msgs`:**
   ```bash
   ros2 pkg create my_python_pkg --build-type ament_python --dependencies rclpy std_msgs
   ```
   > `--build-type ament_python` specifies the build system of the package and that it will be a Python-based ROS 2 package.
   >
   > `--dependencies rclpy std_msgs` specifies the dependencies the package requires:
   >
   > - `rclpy` is the Python client library for ROS 2, needed to write ROS 2 nodes in Python.
   > - `std_msgs` is a package that provides standard message types, such as String, Int32, and Float64
   
3. **Examine the generated package structure** and identify the purpose of each file/directory.

   **Generated Package Structure:**

    ```bash
    my_python_pkg/
    ├── package.xml # file containing meta information about the package
    ├── setup.py # containing instructions for how to install the package
    ├── setup.cfg # required when a package has executables, so ros2 run can find them
    ├── resource/
    │   └── my_python_pkg # marker file for the package
    └── my_python_pkg/ # a directory with the same name as your package, used by ROS 2 tools to find your package, contains __init__.py
        └── __init__.py # used to mark directories on disk as Python package directories
    ```

---

### **Step 2: Write the Publisher Node**

In the `my_python_pkg` folder, create a file named `simple_publisher.py`, which will be the publisher node.

**File:** `~/ros2_ws/src/my_python_pkg/my_python_pkg/simple_publisher.py`

```python
import rclpy                          # Import ROS2 Python client library
from rclpy.node import Node           # Import Node class to create a ROS2 node
from std_msgs.msg import String       # Import standard String message

class SimplePublisher(Node):
    def __init__(self):
        # Initialize the node with the name 'simple_publisher'
        super().__init__('simple_publisher')

        # Create a publisher of message type: String; topic name: 'topic'
        # And queue size: 10 (buffer size for outgoing messages)
        self.publisher_ = self.create_publisher(String, 'ros2intro', 10)

        # Create a timer that triggers 'timer_callback' every 1 second (1Hz)
        # The timer allows us to publish messages at a fixed interval.
        self.timer = self.create_timer(1.0, self.publish_message)  # 1 second interval

        # Initialize a counter to keep track of how many messages we've sent
        self.count = 0

    def publish_message(self):
        # Called every 1.0 second due to the timer.

        # Create a new String message instance
        msg = String()

        # Set its data field to a friendly message including the counter
        msg.data = f'Hello ROS 2: {self.count}'

        # Publish the message to the topic
        self.publisher_.publish(msg)

        # Log output to the console so we see what is being published
        self.get_logger().info(f'Publishing: "{msg.data}"')

        # Increment counter for the next message
        self.count += 1

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)

    # Create an instance of our publisher node
    node = SimplePublisher()

    # Keep the node running until it is killed, allowing callbacks (like the timer) to function
    rclpy.spin(node)
    
    # Once we exit spin (e.g., by pressing Ctrl+C), destroy the node
    node.destroy_node()
    
    # Shutdown ROS2 gracefully
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

### **Step 3: Write the Subscriber Node**

Create the subscriber node as `simple_subscriber.py`.

**File:** `~/ros2_ws/src/my_python_pkg/my_python_pkg/simple_subscriber.py`

```python
import rclpy                          # Import the ROS2 Python client library for initializing nodes and handling ROS-related operations
from rclpy.node import Node           # Import the Node class, which serves as the base class for all ROS2 node implementations
from std_msgs.msg import String       # Import the standard ROS2 String message type for sending and receiving text data

class SimpleSubscriber(Node):
    def __init__(self):
        # Initialize this class as a ROS2 node named 'simple_subscriber'
        super().__init__('simple_subscriber')
        
        # Create a subscription to a topic named 'topic' that publishes String messages.
        # Arguments:
        #   String: The message type the subscriber expects (std_msgs/String)
        #   'topic': The name of the topic to subscribe to
        #   self.listener_callback: The callback function that will be triggered when a new message arrives
        #   10: The queue size (message buffer) if messages arrive faster than they can be processed
        self.subscription = self.create_subscription(
            String,
            'ros2intro',
            self.listener_callback,
            10)
        
        # This line doesn't do anything functionally; it simply prevents a linting or IDE warning about the subscription
        # variable not being used. It's a common practice in ROS2 Python examples.
        self.subscription

    def listener_callback(self, msg):
        # This callback function is triggered each time a new message is received from the 'topic'.
        # 'msg' is a String message, so we access its data field to get the actual text content.
        # We then use the node's built-in logging system to print the received data.
        self.get_logger().info(f'Received: "{msg.data}"')

def main(args=None):
    # Initialize the ROS2 client library, necessary before using any ROS2-related code
    rclpy.init(args=args)
    
    # Create an instance of the SimpleSubscriber node
    node = SimpleSubscriber()

    try:
        # Spin the node, meaning this will block and process any incoming callbacks indefinitely
        # until an external event (like Ctrl+C) interrupts the process.
        rclpy.spin(node)
    except KeyboardInterrupt:
        # If the user presses Ctrl+C in the terminal, a KeyboardInterrupt exception is raised.
        # Here, we simply pass to allow the program to proceed to the 'finally' block for cleanup.
        pass
    finally:
        # Once we are done spinning or have been interrupted, we destroy the node to clean up resources.
        node.destroy_node()
        
        # Shut down the rclpy library, releasing all ROS2-related resources.
        rclpy.shutdown()

# The standard Python entry point for executable scripts.
# If this script is run directly (e.g., 'python3 subscriber.py'), execute the main() function.
if __name__ == '__main__':
    main()
```

> [!Note]
> The `try/except/finally` block is not strictly necessary, but it’s considered good practice for graceful shutdown.

---

### **Step 4: Update the Setup Configuration**

Modify `setup.py` to install your nodes.

**File:** `~/ros2_ws/src/my_python_pkg/setup.py`

```python
from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='A simple publisher and subscriber example in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_python_pkg.simple_publisher:main',
            'simple_subscriber = my_python_pkg.simple_subscriber:main',
        ],
    },
)
```

---

### **Step 5: Build and Source the Package**

Go to the workspace root, build the package, and source it:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

> Replace `.bash` with the appropriate Shell if you are not using Bash Shell

---

### **Step 6: Run the Nodes**

In two separate terminals, run the publisher and subscriber:

#### Terminal 1: Publisher

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_python_pkg simple_publisher
```

#### Terminal 2: Subscriber

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_python_pkg simple_subscriber
```

#### Expected Output

- **Publisher Terminal:**

  ```bash
  [INFO] [simple_publisher]: Publishing: "Hello ROS 2: 0"
  [INFO] [simple_publisher]: Publishing: "Hello ROS 2: 1"
  ...
  ```

- **Subscriber Terminal:**

  ```bash
  [INFO] [simple_subscriber]: Received: "Hello ROS 2: 0"
  [INFO] [simple_subscriber]: Received: "Hello ROS 2: 1"
  ...
  ```

---

## Track B - C++ Publisher-Subscriber Implementation

To implement the **publisher and subscriber in C++** transmitting standard `Strings`, follow the steps below.

### **Step 1: Create a C++ Package**

Navigate to the `src/` directory:

```bash
cd ~/ros2_ws/src
```

Create a C++ package named **`my_cpp_pkg`**:

```bash
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
```

This creates the following structure:

```bash
my_cpp_pkg/
├── CMakeLists.txt # file that describes how to build the code within the package
├── include/<package_name> # directory containing the public headers for the package
├── package.xml # file containing meta information about the package
└── src/ # directory containing the source code for the package
```

---

### **Step 2: Write the Publisher Node (C++)**

Create the file `simple_publisher.cpp` inside the `src` directory of the `my_cpp_pkg` package.

**File:** `~/ros2_ws/src/my_cpp_pkg/src/simple_publisher.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimplePublisher : public rclcpp::Node {
public:
    SimplePublisher() : Node("simple_publisher"), count_(0) {
        publisher_ = this->create_publisher<std_msgs::msg::String>("ros2intro", 10);
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SimplePublisher::publish_message, this));
    }

private:
    void publish_message() {
        auto message = std_msgs::msg::String();
        message.data = "Hello ROS 2: " + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimplePublisher>());
    rclcpp::shutdown();
    return 0;
}
```

---

### **Step 3: Write the Subscriber Node (C++)**

Create the file `simple_subscriber.cpp` inside the `src` directory of the `my_cpp_pkg` package.

**File:** `~/ros2_ws/src/my_cpp_pkg/src/simple_subscriber.cpp`

```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class SimpleSubscriber : public rclcpp::Node {
public:
    SimpleSubscriber() : Node("simple_subscriber") {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "ros2intro", 10, std::bind(&SimpleSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const {
        RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSubscriber>());
    rclcpp::shutdown();
    return 0;
}
```

---

### **Step 4: Update `CMakeLists.txt`**

Modify `CMakeLists.txt` to compile both the publisher and subscriber nodes.

**File:** `~/ros2_ws/src/my_cpp_pkg/CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(simple_publisher src/simple_publisher.cpp)
ament_target_dependencies(simple_publisher rclcpp std_msgs)

add_executable(simple_subscriber src/simple_subscriber.cpp)
ament_target_dependencies(simple_subscriber rclcpp std_msgs)

install(TARGETS
  simple_publisher
  simple_subscriber
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

---

### **Step 5: Build and Source the Package (C++)**

Build the package and source it:

```bash
cd ~/ros2_ws
colcon build --packages-select my_cpp_pkg
source install/setup.bash
```

---

### **Step 6: Run the Nodes (C++)**

Open two terminals and run the publisher and subscriber nodes.

#### Terminal 1: Run the Publisher**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_cpp_pkg simple_publisher
```

#### Terminal 2: Run the Subscriber**

```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_cpp_pkg simple_subscriber
```

#### Expected Output (C++)

- **Publisher Terminal:**

  ```bash
  [INFO] [simple_publisher]: Publishing: 'Hello ROS 2: 0'
  [INFO] [simple_publisher]: Publishing: 'Hello ROS 2: 1'
  ...
  ```

- **Subscriber Terminal:**

  ```bash
  [INFO] [simple_subscriber]: Received: 'Hello ROS 2: 0'
  [INFO] [simple_subscriber]: Received: 'Hello ROS 2: 1'
  ...
  ```


## Required Behavior

* One node publishes a message at 1 Hz
* One node subscribes and prints received messages
* Topic name: `/ros2intro`

## Concept Reinforcement

* Nodes = independent processes
* Topics = anonymous data buses
* Messages = typed data structures

---

# **Part 2 - ROS 2 CLI Introspection**

## Objective

Learn how to **debug and inspect live systems**

## Tasks

With your nodes running, open a new terminal and execute:

1. **List all active nodes:**
   ```bash
   ros2 node list
   ```

2. **Get detailed information about a node:**
   ```bash
   ros2 node info /simple_publisher
   ros2 node info /simple_subscriber
   ```

3. **List all active topics:**
   ```bash
   ros2 topic list
   ```

4. **Get information about the `ros2intro` topic:**
   ```bash
   ros2 topic info /ros2intro
   ```

5. **Echo messages from the topic:**
   ```bash
   ros2 topic echo /ros2intro
   ```

6. **Check the publishing rate:**
   ```bash
   ros2 topic hz /ros2intro
   ```


---

# **Part 3 - Custom Message Types**

## Overview

Custom message types allow you to define your own data structures for communication between nodes. Instead of using only standard messages like String or Float32, you can create messages that bundle multiple fields together, making your code more organized and meaningful.

## Understanding Custom Messages

### Why Use Custom Messages?

**Standard Message:**
```python
# Publishing speed as a Float32
speed_msg = Float32()
speed_msg.data = 25.5
```

**Custom Message:**
```python
# Publishing complete vehicle state
vehicle_msg = VehicleState()
vehicle_msg.speed = 25.5
vehicle_msg.steering_angle = 15.0
vehicle_msg.throttle = 0.8
vehicle_msg.timestamp = self.get_clock().now().to_msg()
```

Custom messages are more **descriptive**, **organized**, and **maintainable** for complex robotic applications.

---

## Creating a Custom Message - Step by Step

### Step 1: Create a Package for Custom Messages

It's best practice to create a separate package for custom messages so multiple packages can use them.

```bash
cd ~/ros2_ws/src
ros2 pkg create my_custom_msgs --build-type ament_cmake
```

**Why `ament_cmake`?** Message generation requires CMake build system, even if your nodes are in Python.

### Step 2: Create Message Directory and File

```bash
cd ~/ros2_ws/src/my_custom_msgs
mkdir msg
```

Create your first custom message file:

**File:** `~/ros2_ws/src/my_custom_msgs/msg/VehicleState.msg`

```
# VehicleState.msg
# Custom message for vehicle state information

float32 speed                  # Vehicle speed in m/s
float32 steering_angle        # Steering angle in degrees
float32 throttle              # Throttle position (0.0 to 1.0)
builtin_interfaces/Time timestamp  # Timestamp of the measurement
```

**Message Field Types:**
- Basic types: `bool`, `int8`, `int16`, `int32`, `int64`, `float32`, `float64`, `string`
- Arrays: `float32[]` (dynamic), `float32[5]` (fixed size)
- Other messages: `builtin_interfaces/Time`, `std_msgs/Header`

### Step 3: Update package.xml

Add the required dependencies to `~/ros2_ws/src/my_custom_msgs/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_custom_msgs</name>
  <version>0.0.0</version>
  <description>Custom message types for ROS2 tutorial</description>
  <maintainer email="student@example.com">student</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <!-- Add these dependencies for message generation -->
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  
  <!-- Add dependency on builtin_interfaces for Time type -->
  <depend>builtin_interfaces</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

### Step 4: Update CMakeLists.txt

Update `~/ros2_ws/src/my_custom_msgs/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(my_custom_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Declare the message files to generate code for
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VehicleState.msg"
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

### Step 5: Build the Message Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_custom_msgs
source install/setup.bash
```

**Expected Output:**
```
Starting >>> my_custom_msgs
Finished <<< my_custom_msgs [5.23s]

Summary: 1 package finished [5.67s]
```

### Step 6: Verify Message Creation

Check that your message was created:

```bash
ros2 interface show my_custom_msgs/msg/VehicleState
```

**Expected Output:**
```
float32 speed
float32 steering_angle
float32 throttle
builtin_interfaces/Time timestamp
        int32 sec
        uint32 nanosec
```

---

## Using Custom Messages in Python Nodes

### Step 1: Update Python Package Dependencies

Update `~/ros2_ws/src/my_python_pkg/package.xml` to add the custom message dependency:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_python_pkg</name>
  <version>0.0.0</version>
  <description>A simple publisher and subscriber example in ROS 2</description>
  <maintainer email="student@example.com">student</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>my_custom_msgs</depend>  <!-- Add this line -->

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 2: Create Custom Message Publisher

**File:** `~/ros2_ws/src/my_python_pkg/my_python_pkg/custom_publisher.py`

```python
import rclpy
from rclpy.node import Node
from my_custom_msgs.msg import VehicleState
import random
import math

class CustomPublisher(Node):
    def __init__(self):
        super().__init__('custom_publisher')
        
        # Create publisher for custom message type
        self.publisher_ = self.create_publisher(
            VehicleState, 
            'vehicle_state', 
            10)
        
        # Timer to publish at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_vehicle_state)
        
        # Simulation variables
        self.simulation_time = 0.0
        
        self.get_logger().info('Custom Publisher started - Publishing VehicleState messages')

    def publish_vehicle_state(self):
        # Create custom message instance
        msg = VehicleState()
        
        # Simulate varying vehicle state
        self.simulation_time += 0.5
        
        # Speed varies between 10-30 m/s with some randomness
        msg.speed = 20.0 + 10.0 * math.sin(self.simulation_time * 0.5) + random.uniform(-1, 1)
        
        # Steering angle varies between -30 and +30 degrees
        msg.steering_angle = 15.0 * math.sin(self.simulation_time * 0.3) + random.uniform(-2, 2)
        
        # Throttle varies between 0.3 and 1.0
        msg.throttle = 0.65 + 0.35 * math.sin(self.simulation_time * 0.4)
        
        # Add timestamp
        msg.timestamp = self.get_clock().now().to_msg()
        
        # Publish the message
        self.publisher_.publish(msg)
        
        # Log the published data
        self.get_logger().info(
            f'Publishing VehicleState: '
            f'speed={msg.speed:.2f} m/s, '
            f'steering={msg.steering_angle:.2f}°, '
            f'throttle={msg.throttle:.2f}'
        )

def main(args=None):
    rclpy.init(args=args)
    node = CustomPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 3: Create Custom Message Subscriber

**File:** `~/ros2_ws/src/my_python_pkg/my_python_pkg/custom_subscriber.py`

```python
import rclpy
from rclpy.node import Node
from my_custom_msgs.msg import VehicleState

class CustomSubscriber(Node):
    def __init__(self):
        super().__init__('custom_subscriber')
        
        # Create subscription for custom message type
        self.subscription = self.create_subscription(
            VehicleState,
            'vehicle_state',
            self.vehicle_state_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        
        self.get_logger().info('Custom Subscriber started - Listening for VehicleState messages')

    def vehicle_state_callback(self, msg):
        # Extract timestamp
        timestamp_sec = msg.timestamp.sec + msg.timestamp.nanosec * 1e-9
        
        # Analyze the vehicle state
        speed_status = self.analyze_speed(msg.speed)
        steering_status = self.analyze_steering(msg.steering_angle)
        throttle_status = self.analyze_throttle(msg.throttle)
        
        # Log received data with analysis
        self.get_logger().info(
            f'Received VehicleState:\n'
            f'  Speed: {msg.speed:.2f} m/s ({speed_status})\n'
            f'  Steering: {msg.steering_angle:.2f}° ({steering_status})\n'
            f'  Throttle: {msg.throttle:.2f} ({throttle_status})\n'
            f'  Timestamp: {timestamp_sec:.2f}s'
        )
    
    def analyze_speed(self, speed):
        """Provide context about speed"""
        if speed < 15:
            return "SLOW"
        elif speed < 25:
            return "MODERATE"
        else:
            return "FAST"
    
    def analyze_steering(self, angle):
        """Provide context about steering"""
        if abs(angle) < 5:
            return "STRAIGHT"
        elif abs(angle) < 15:
            return "GENTLE TURN"
        else:
            return "SHARP TURN"
    
    def analyze_throttle(self, throttle):
        """Provide context about throttle"""
        if throttle < 0.4:
            return "LOW"
        elif throttle < 0.7:
            return "MEDIUM"
        else:
            return "HIGH"

def main(args=None):
    rclpy.init(args=args)
    node = CustomSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Step 4: Update setup.py

Update `~/ros2_ws/src/my_python_pkg/setup.py` to include the new executables:

```python
from setuptools import setup

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='A simple publisher and subscriber example in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_python_pkg.simple_publisher:main',
            'simple_subscriber = my_python_pkg.simple_subscriber:main',
            'custom_publisher = my_python_pkg.custom_publisher:main',
            'custom_subscriber = my_python_pkg.custom_subscriber:main',
        ],
    },
)
```

### Step 5: Build and Test

```bash
cd ~/ros2_ws
colcon build --packages-select my_python_pkg
source install/setup.bash
```

**Terminal 1 - Run Custom Publisher:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_python_pkg custom_publisher
```

**Expected Output:**
```
[INFO] [custom_publisher]: Custom Publisher started - Publishing VehicleState messages
[INFO] [custom_publisher]: Publishing VehicleState: speed=20.45 m/s, steering=2.34°, throttle=0.78
[INFO] [custom_publisher]: Publishing VehicleState: speed=24.23 m/s, steering=5.67°, throttle=0.85
[INFO] [custom_publisher]: Publishing VehicleState: speed=28.91 m/s, steering=-1.23°, throttle=0.92
[INFO] [custom_publisher]: Publishing VehicleState: speed=26.45 m/s, steering=-8.45°, throttle=0.88
```

**Terminal 2 - Run Custom Subscriber:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 run my_python_pkg custom_subscriber
```

**Expected Output:**
```
[INFO] [custom_subscriber]: Custom Subscriber started - Listening for VehicleState messages
[INFO] [custom_subscriber]: Received VehicleState:
  Speed: 20.45 m/s (MODERATE)
  Steering: 2.34° (STRAIGHT)
  Throttle: 0.78 (HIGH)
  Timestamp: 1735411234.56s
[INFO] [custom_subscriber]: Received VehicleState:
  Speed: 24.23 m/s (MODERATE)
  Steering: 5.67° (GENTLE TURN)
  Throttle: 0.85 (HIGH)
  Timestamp: 1735411235.06s
```

---

# **Part 4 - Launch Files (Multi-Node Systems)**

## Overview

Launch files allow you to start multiple nodes simultaneously with a single command. Instead of opening multiple terminals and running each node individually, you can configure and launch your entire system at once. This is essential for complex robotic systems

## Understanding Launch Files

### Why Use Launch Files?

**Without Launch Files:**
```bash
# Terminal 1
ros2 run my_python_pkg simple_publisher

# Terminal 2
ros2 run my_python_pkg simple_subscriber

# Terminal 3
ros2 run my_python_pkg custom_publisher

# Terminal 4
ros2 run my_python_pkg custom_subscriber
```

**With Launch Files:**
```bash
# Single terminal
ros2 launch my_python_pkg all_nodes_launch.py
```

### Benefits of Launch Files:
- Start multiple nodes with one command
- Configure parameters for nodes
- Set remapping rules for topics
- Organize complex systems
- Reproducible system startup
- Easier testing and debugging

---

## Python Launch Files

### Step 1: Create Launch Directory

```bash
cd ~/ros2_ws/src/my_python_pkg
mkdir launch
```

### Step 2: Create Simple Launch File

**File:** `~/ros2_ws/src/my_python_pkg/launch/simple_nodes_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch simple publisher and subscriber nodes"""
    
    return LaunchDescription([
        # Launch simple publisher node
        Node(
            package='my_python_pkg',
            executable='simple_publisher',
            name='simple_publisher',
            output='screen',
            emulate_tty=True,
        ),
        
        # Launch simple subscriber node
        Node(
            package='my_python_pkg',
            executable='simple_subscriber',
            name='simple_subscriber',
            output='screen',
            emulate_tty=True,
        ),
    ])
```

**Explanation of Parameters:**
- `package`: The ROS2 package containing the executable
- `executable`: The name of the node executable (from setup.py entry_points)
- `name`: The name the node will use (can override the node's internal name)
- `output='screen'`: Display node output in the terminal
- `emulate_tty=True`: Ensures colored output and proper formatting


### Step 3: Create Custom Messages Launch File

**File:** `~/ros2_ws/src/my_python_pkg/launch/custom_nodes_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launch custom message publisher and subscriber nodes"""
    
    return LaunchDescription([
        # Launch custom publisher node
        Node(
            package='my_python_pkg',
            executable='custom_publisher',
            name='custom_publisher',
            output='screen',
            emulate_tty=True,
        ),
        
        # Launch custom subscriber node
        Node(
            package='my_python_pkg',
            executable='custom_subscriber',
            name='custom_subscriber',
            output='screen',
            emulate_tty=True,
        ),
    ])
```

### Step 4: Create Comprehensive Launch File (All Nodes)

**File:** `~/ros2_ws/src/my_python_pkg/launch/all_nodes_launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch all publisher and subscriber nodes:
    - Simple string message nodes
    - Custom VehicleState message nodes
    """
    
    return LaunchDescription([
        # Log info message
        LogInfo(msg="Starting all ROS2 tutorial nodes..."),
        
        # Simple Publisher (String messages)
        Node(
            package='my_python_pkg',
            executable='simple_publisher',
            name='simple_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[],
        ),
        
        # Simple Subscriber (String messages)
        Node(
            package='my_python_pkg',
            executable='simple_subscriber',
            name='simple_subscriber',
            output='screen',
            emulate_tty=True,
            parameters=[],
        ),
        
        # Custom Publisher (VehicleState messages)
        Node(
            package='my_python_pkg',
            executable='custom_publisher',
            name='custom_publisher',
            output='screen',
            emulate_tty=True,
            parameters=[],
        ),
        
        # Custom Subscriber (VehicleState messages)
        Node(
            package='my_python_pkg',
            executable='custom_subscriber',
            name='custom_subscriber',
            output='screen',
            emulate_tty=True,
            parameters=[],
        ),
        
        LogInfo(msg="All nodes launched successfully!"),
    ])
```

### Step 5: Update setup.py to Include Launch Files

**File:** `~/ros2_ws/src/my_python_pkg/setup.py`

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_python_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='A simple publisher and subscriber example in ROS 2',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = my_python_pkg.simple_publisher:main',
            'simple_subscriber = my_python_pkg.simple_subscriber:main',
            'custom_publisher = my_python_pkg.custom_publisher:main',
            'custom_subscriber = my_python_pkg.custom_subscriber:main',
        ],
    },
)
```

### Step 6: Update package.xml for Launch Dependencies

**File:** `~/ros2_ws/src/my_python_pkg/package.xml`

Add these dependencies if not already present:

```xml
  <exec_depend>ros2launch</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
```

Complete package.xml should look like:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_python_pkg</name>
  <version>0.0.0</version>
  <description>A simple publisher and subscriber example in ROS 2</description>
  <maintainer email="student@example.com">student</maintainer>
  <license>Apache License 2.0</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>my_custom_msgs</depend>
  
  <!-- Launch file dependencies -->
  <exec_depend>ros2launch</exec_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### Step 7: Build and Test Launch Files

```bash
cd ~/ros2_ws
colcon build --packages-select my_python_pkg
source install/setup.bash
```

### Step 8: Run Launch Files

**Launch simple nodes only:**
```bash
ros2 launch my_python_pkg simple_nodes_launch.py
```

**Expected Output:**
```
[INFO] [launch]: All log files can be found below /home/student/.ros/log/2024-01-02-12-34-56-789012-ubuntu-12345
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [simple_publisher-1]: process started with pid [12346]
[INFO] [simple_subscriber-2]: process started with pid [12347]
[simple_publisher-1] [INFO] [1735411234.123456789] [simple_publisher]: Publishing: "Hello ROS 2: 0"
[simple_subscriber-2] [INFO] [1735411234.123456789] [simple_subscriber]: Received: "Hello ROS 2: 0"
[simple_publisher-1] [INFO] [1735411235.123456789] [simple_publisher]: Publishing: "Hello ROS 2: 1"
[simple_subscriber-2] [INFO] [1735411235.123456789] [simple_subscriber]: Received: "Hello ROS 2: 1"
```

**Launch custom message nodes only:**
```bash
ros2 launch my_python_pkg custom_nodes_launch.py
```

**Launch all nodes:**
```bash
ros2 launch my_python_pkg all_nodes_launch.py
```

**Expected Output:**
```
[INFO] [launch]: All log files can be found below /home/student/.ros/log/2024-01-02-12-34-56-789012-ubuntu-12345
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [launch.user]: Starting all ROS2 tutorial nodes...
[INFO] [simple_publisher-1]: process started with pid [12346]
[INFO] [simple_subscriber-2]: process started with pid [12347]
[INFO] [custom_publisher-3]: process started with pid [12348]
[INFO] [custom_subscriber-4]: process started with pid [12349]
[INFO] [launch.user]: All nodes launched successfully!
[simple_publisher-1] [INFO] [1735411234.123456789] [simple_publisher]: Publishing: "Hello ROS 2: 0"
[simple_subscriber-2] [INFO] [1735411234.123456789] [simple_subscriber]: Received: "Hello ROS 2: 0"
[custom_publisher-3] [INFO] [1735411234.567890123] [custom_publisher]: Publishing VehicleState: speed=22.45 m/s, steering=3.21°, throttle=0.78
[custom_subscriber-4] [INFO] [1735411234.567890123] [custom_subscriber]: Received VehicleState:
[custom_subscriber-4]   Speed: 22.45 m/s (MODERATE)
[custom_subscriber-4]   Steering: 3.21° (STRAIGHT)
[custom_subscriber-4]   Throttle: 0.78 (HIGH)
[custom_subscriber-4]   Timestamp: 1735411234.57s
```

**To stop all nodes:** Press `Ctrl+C` in the terminal

---

# **Deliverable**

## Multi-Node Communication System - Data Processing Pipeline

Design and implement a simple multi-node system with the following architecture:

```
[Sensor Node] --> /raw_data --> [Processing Node] --> /processed_data --> [Display Node]
```

### **Requirements:**

1. **Sensor Node**: Publishes simulated or random sensor readings (e.g., temperature values, speed, etc..) to `/raw_data`
2. **Processing Node**: Subscribes to `/raw_data`, applies a simple transformation (e.g., Celsius to Fahrenheit, kilometers to miles, etc..), and publishes to `/processed_data`
3. **Display Node**: Subscribes to `/processed_data` and displays the results
4. **Custom Data Type**: Create at least one custom data types to be used by one or more of the nodes
5. **Launch File**: Create a launch file that runs all of the required nodes and set parameters if needed
6. **Screenshots**: Collect screenshots showing the entire system running properly

### **Implementation Notes:**

- You can implement this in Python or C++ (or mix both)
- Create these nodes in your existing package or create a new package
- Use appropriate message types (e.g., `std_msgs/Float32` for numeric data)

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
   first-name_last-name_studentID.zip
   ```

   Example:

   ```
   jane_doe_12345678.zip
   ```

### Required Items

1. **Source Code**

   * Publisher and subscriber nodes
   * Custom message definition
   * Launch file

2. **Screenshots**
   Withing the `src` folder, create a folder named `screenshots` and include the output of the following:
   
   * `ros2 node list`
   * `ros2 topic echo`
   * Successful launch file execution


## **Grading Rubric (100 Points)**

| Category                           | Points |
| ---------------------------------- | ------ |
| Publisher/subscriber functionality | 30     |
| Custom message implementation      | 25     |
| Launch file (multi-node system)    | 20     |
| ROS 2 CLI usage & screenshots      | 15     |
| Well organized submission, well formatted codes          | 10     |

