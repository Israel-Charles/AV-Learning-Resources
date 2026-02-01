# Assignment Helper: Integrated TurtleSim Visualization and Recording System

This document is to help you complete the deliverable: **Integrated TurtleSim Visualization and Recording System**

## Key Concepts You Need to Understand

Before writing your launch file, make sure you understand these core ideas:

**What is a ROS2 Launch File?**
A launch file is a Python script that orchestrates the startup of multiple ROS2 nodes and processes simultaneously. Instead of opening 6+ terminals manually, one launch file handles everything. The entry point is always a function called `generate_launch_description()` that returns a `LaunchDescription` object containing a list of actions.

**What are Launch Actions?**
Every item in your launch file is an "action" — an instruction telling ROS2 to do something. The most common ones you'll use are listed in the resources below.

---

## Resource 0: Installing `xterm`

To simplify working with teleoperation features-especially when using launch files-it’s recommended to install a terminal emulator such as `xterm`. You can install it with:

```bash
sudo apt install xterm
```

> **Note:** You may also use any other terminal emulator or terminal multiplexer (e.g., `gnome-terminal`, `tmux`) if you prefer.

---

## Resource 1: Import Statements You Will Need

These are the building blocks. You won't necessarily need all of them, but here's what each one does:

```python
#!/usr/bin/env python3
import os  # For file path manipulation (joining paths, etc.)

from launch import LaunchDescription          # The container that holds all your actions
from launch.actions import (
    DeclareLaunchArgument,       # Declares a configurable parameter (e.g., record duration)
    ExecuteProcess,              # Runs any shell/terminal command
    LogInfo,                     # Prints a message to the terminal when the launch file runs
    TimerAction,                 # Delays execution of actions by a number of seconds
    SetEnvironmentVariable,      # Sets an env var so that bash scripts inside ExecuteProcess can read it
    IncludeLaunchDescription,    # Includes/runs another launch file inside yours
)
from launch.launch_description_sources import PythonLaunchDescriptionSource  # Needed by IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration   # References a declared launch argument by name
from launch_ros.substitutions import FindPackageShare  # Finds the installed path of a ROS package
from launch_ros.actions import Node                    # Launches a ROS2 node
```

---

## Resource 2: Declaring and Using Launch Arguments

Launch arguments let users configure values without editing code. You declare them, then reference them:

```python
# --- STEP 1: Declare the argument (goes inside generate_launch_description, before the return) ---
DeclareLaunchArgument(
    "record_seconds",                          # The argument name (used on command line)
    default_value="20",                        # Default if user doesn't specify
    description="How many seconds to record.", # Shown in --help
)

# --- STEP 2: Create a reference to it ---
record_seconds = LaunchConfiguration("record_seconds")
# Now `record_seconds` is a substitution object — it resolves to the actual value at runtime.

# --- STEP 3: Pass it to bash via environment variable (since bash can't read LaunchConfiguration directly) ---
SetEnvironmentVariable("TS_DEMO_RECORD_SECONDS", record_seconds)
# Now in any ExecuteProcess bash script, you can use $TS_DEMO_RECORD_SECONDS
```

**Usage on command line:**
```bash
ros2 launch turtlesim_demo demo.launch.py record_seconds:=30
```

---

## Resource 3: Launching ROS2 Nodes

Use the `Node` action for any ROS2 node. The `prefix` field is a powerful trick to open a node in its own terminal window:

```python
# --- Basic node (output goes to the main launch terminal) ---
Node(
    package="turtlesim",          # The ROS package the node belongs to
    executable="turtlesim_node",  # The actual node executable
    name="turtlesim",             # The name this node registers as in the ROS graph
    output="screen",              # Print its output to the terminal
)

# --- Node in its own xterm window (great for interactive nodes like teleop) ---
Node(
    package="turtlesim",
    executable="turtle_teleop_key",
    name="teleop_turtle1",
    output="screen",
    prefix="xterm -title 'Teleop Turtle1 (Arrow Keys)' -e",
    # `prefix` prepends this command before the node executable.
    # xterm opens a new window; -title sets the window title; -e runs the command inside it.
)

# --- Node with command-line arguments (e.g., RViz with a config file) ---
Node(
    package="rviz2",
    executable="rviz2",
    name="rviz2",
    arguments=["-d", rviz_config],  # -d flag loads a .rviz config file
    output="screen",
)
```

---

## Resource 4: Running Shell Commands with ExecuteProcess

`ExecuteProcess` runs any shell command. The `bash -lc` pattern is important — it launches a login shell so ROS2 environment variables (and your custom env vars) are available:

```python
# --- Simple command in its own xterm ---
ExecuteProcess(
    cmd=[
        "bash", "-lc",
        "xterm -title 'My Window' -e ros2 run tf2_ros tf2_echo world turtle1",
    ],
    output="screen",
)

# --- Multi-line bash script with env var access ---
# Note: Use r""" (raw string) to avoid issues with backslashes
ExecuteProcess(
    cmd=[
        "bash", "-lc",
        r"""
        echo "Starting something...";
        echo "The record time is: $TS_DEMO_RECORD_SECONDS seconds";
        # Your commands here — env vars from SetEnvironmentVariable are available
        timeout "$TS_DEMO_RECORD_SECONDS" ros2 bag record /turtle1/cmd_vel -o /tmp/my_bag
        """,
    ],
    output="screen",
)

# --- xterm with -hold flag (keeps window open even if command finishes/errors) ---
ExecuteProcess(
    cmd=[
        "xterm", "-hold", "-title", "My Tool",
        "-e", "bash", "-lc",
        "echo 'Running tool...'; rqt --standalone rqt_plot /turtle1/pose/x;",
    ],
    output="screen",
)
```

**Key `xterm` flags:**
| Flag | What it does |
|---|---|
| `-title 'Name'` | Sets the window title |
| `-e` | Runs the following command inside the xterm |
| `-hold` | Keeps the window open after the command exits (useful for seeing errors) |

---

## Resource 5: Including Another Launch File

This is how you bring in the `turtle_tf2_demo` launch file, which automatically spawns `turtle2` and sets up TF frames for both turtles:

```python
# --- STEP 1: Find the other package's launch file path ---
turtle_tf2_pkg = FindPackageShare("turtle_tf2_py").find("turtle_tf2_py")
turtle_tf2_demo_launch = os.path.join(turtle_tf2_pkg, "launch", "turtle_tf2_demo.launch.py")

# --- STEP 2: Include it ---
tf2_demo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(turtle_tf2_demo_launch)
)
# This runs everything that launch file defines — you get turtle2, TF frames, etc. for free.
```

> **Why use this instead of launching nodes manually?**
> The `turtle_tf2_demo` launch file already handles spawning turtle2 and broadcasting TF transforms for both turtles. Including it saves you from replicating that setup.

---

## Resource 6: Delayed Execution with TimerAction

`TimerAction` waits for a specified period before running a set of actions. This is critical for sequencing — you need all nodes to be up before recording starts:

```python
# This waits 3 seconds after launch, THEN runs the actions inside it
start_recording = TimerAction(
    period=3.0,                  # Wait 3 seconds
    actions=[                    # Then run these actions:
        LogInfo(msg="Recording phase begins now."),
        # ... your recording ExecuteProcess actions ...
    ],
)
```

**Important:** `TimerAction` does NOT block other actions. Everything else in your `LaunchDescription` starts immediately. The timer just delays *its own* actions.

---

## Resource 7: Printing Messages with LogInfo

`LogInfo` prints a message to the launch terminal. Great for indicating phases and giving the user feedback:

```python
LogInfo(msg="=== PHASE: RECORDING ==="),
LogInfo(msg="Drive turtle1 with the teleop window now!"),
LogInfo(msg="Recording will last 20 seconds."),
```

---

## Resource 8: Recording and Playback Command Reference

These are the exact `ros2 bag` commands you need and what each flag does:

```bash
# --- RECORDING ---
# Record a specific topic, save to a directory, auto-stop after N seconds
timeout 20 ros2 bag record /turtle1/cmd_vel -o /tmp/turtlesim_demo/my_bag
#  ^^^^^^^                                    ^^^^^^^^^^^^^^^^^^^^^^^^^^
#  Kills the command after 20s               Output directory for the bag

# --- PLAYBACK ---
# Play back a recorded bag at 2x speed
ros2 bag play /tmp/turtlesim_demo/my_bag --rate 2.0
#                                        ^^^^^^^^^^^
#                                        Playback speed multiplier
```

**Sequencing trick:** For playback to happen *after* recording finishes, you need to account for:
1. The startup delay before recording begins
2. The recording duration itself
3. A small buffer for the bag file to finalize

You can use `sleep` commands inside a bash script to achieve this timing.

---

## Resource 9: rqt Launch Command Reference

```bash
# --- rqt_plot with specific topics (programmatic, no config file needed) ---
rqt -s rqt_plot --args /turtle1/pose/x /turtle1/pose/y /turtle1/pose/linear_velocity

# --- rqt standalone plugin ---
rqt --standalone rqt_tf_tree
```

| Flag | Meaning |
|---|---|
| `-s rqt_plot` | Start rqt with the rqt_plot plugin selected |
| `--args` | Everything after this is passed as arguments to the plugin |
| `--standalone` | Launch a single plugin in its own window without the full rqt shell |

---

## Resource 10: RViz Configuration File (`.rviz`) Structure

If you choose **Option A** (pre-configured files), here's what a `.rviz` file looks like and what the key fields control:

```yaml
Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/TF       # Displays coordinate frames
      Enabled: true
      Frames:
        All Enabled: true                  # Show ALL tf frames (turtle1, turtle2, world, etc.)
      Show Arrows: true
      Show Axes: true
      Show Names: true                     # Label each frame

    - Class: rviz_default_plugins/Grid     # A ground-plane grid for spatial reference
      Enabled: true
      Plane: XY

  Global Options:
    Fixed Frame: turtle2                   # The "camera" is anchored to turtle2
    # Setting Fixed Frame to turtle2 means turtle1's motion is shown RELATIVE to turtle2.
    # This is what makes RViz show the relative movement between the two turtles.
    Frame Rate: 30

  Views:
    Current:
      Class: rviz_default_plugins/Orbit    # Orbit camera (you can rotate the view)
      Distance: 16                         # How far the camera is from the focal point
      Focal Point:                         # What the camera looks at
        X: 0
        Y: 0
        Z: 0
      Pitch: 0.785                         # ~45 degrees (top-down-ish view)
      Yaw: 0.785

Window Geometry:                           # Initial window size and position
  Height: 900
  Width: 1400
```

> **Key insight:** Setting `Fixed Frame: turtle2` is what makes RViz show turtle1's movement *relative* to turtle2. If you set it to `world`, both turtles move independently on a static grid.

---

## Resource 11: Putting It All Together — The Structure

Here's the skeleton of how your `generate_launch_description()` function should be organized. The **order inside the returned list matters for readability**, but most actions start simultaneously (except `TimerAction`-delayed ones):

```python
def generate_launch_description():

    # 1. Declare launch arguments (record duration, bag directory, etc.)
    # 2. Create LaunchConfiguration references to those arguments
    # 3. Set up file paths (rviz config, included launch files)
    # 4. Define all your actions (nodes, processes, timers, log messages)
    # 5. Return LaunchDescription with everything in a logical order:

    return LaunchDescription([
        # A) Argument declarations (must come first)
        DeclareLaunchArgument(...),

        # B) Environment variables (so bash scripts can access args)
        SetEnvironmentVariable(...),

        # C) Info messages (printed immediately on launch)
        LogInfo(...),

        # D) The included tf2 demo launch (spawns turtle2 + TF)
        tf2_demo,

        # E) Visualization tools (rviz, rqt — all start immediately)
        rviz2,
        rqt_plot,

        # F) User control (teleop)
        teleop,

        # G) Orchestration (timer-delayed recording, then playback)
        start_recording,   # TimerAction: waits, then starts recording
        delayed_playback,  # ExecuteProcess: sleeps internally, then plays back
    ])
```

---

## Tips & Common Pitfalls

**Timing is tricky.** Your playback script needs to sleep long enough to cover: the timer delay before recording starts + the full recording duration + a small buffer. If playback starts too early, the bag file won't exist yet or will be incomplete.

**Environment variables are your bridge.** You can't directly pass a `LaunchConfiguration` into a bash script. Use `SetEnvironmentVariable` to export it, then read it as `$YOUR_VAR_NAME` inside bash.

**`bash -lc` is essential.** The `-l` flag makes it a login shell, which sources your ROS2 environment. Without it, commands like `ros2` won't be found.

**`xterm` is optional but helpful. You could use other tools like `xterm`. Without a tool like that, you might have an hard time getting the teleop to work properly.** You could put everything in the main terminal with `output="screen"`, but separate xterm windows make it much easier to see what's happening in each process, especially for interactive tools like teleop.

**Option A vs Option B.** Option A (config files) is cleaner for RViz since `.rviz` files give you fine-grained control over the camera, displays, and layout. Option B (programmatic) is simpler for rqt since you just pass topic names as command-line arguments. You can mix and match.
