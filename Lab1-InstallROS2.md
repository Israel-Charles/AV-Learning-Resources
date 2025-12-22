# Lab 1 – Installing ROS 2 Humble on Ubuntu 22.04

## 0. Verify Ubuntu Version (Required)

ROS 2 **Humble** officially supports **Ubuntu 22.04**.

Open a terminal and run:

```bash
lsb_release -a
```

You should see:

```
Distributor ID: Ubuntu
Release:        22.04
```

If not, **stop** and fix your Ubuntu installation before continuing.

---

## 1. Update System Packages

Always start with a clean, updated system.

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y curl software-properties-common
```

Reboot if the system asks you to.

---

## 2. Enable Ubuntu Universe Repository

ROS 2 packages are hosted in the **universe** repository.

```bash
sudo add-apt-repository universe
sudo apt update
```

---

## 3. Add the ROS 2 GPG Key

This step allows Ubuntu to trust ROS 2 packages.

```bash
sudo mkdir -p /etc/apt/keyrings
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | \
sudo gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg
```

---

## 4. Add the ROS 2 Repository

This tells Ubuntu where to download ROS 2 from.

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

Update package lists again:

```bash
sudo apt update
```

---

## 5. Install ROS 2 Humble

Install the **desktop** version (includes RViz, demos, and tools).

```bash
sudo apt install -y ros-humble-desktop
```

> This works on **x86_64 (Intel/AMD)** and **ARM64 (Apple Silicon)**.

---

## 6. Install Development Tools

These tools are required for building ROS 2 packages in later labs.

```bash
sudo apt install ros-dev-tools
```

---

## 7. Initialize rosdep (Required)

`rosdep` installs system dependencies for ROS packages.

```bash
sudo rosdep init
rosdep update
```

If `rosdep init` says it already exists, that is OK.

---

## 8. Source ROS 2 Automatically

You must source ROS every time you open a terminal.
We will automate this.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

> One important caution
> If you later install multiple ROS versions, auto-sourcing one distro can cause conflicts.
> In that case, you might:
> Comment it out in .bashrc, or
> Source the one you want manually
> For a single-distro setup, this is perfectly fine.

---

## 9. Verify ROS 2 Installation

### Check ROS version:

```bash
ros2 --version
```

You should see output indicating ROS 2 is installed.

### Run a Talker/Listener Test:

Open **Terminal 1**:

```bash
ros2 run demo_nodes_cpp talker
```

Open **Terminal 2**:

```bash
ros2 run demo_nodes_py listener
```

If messages appear in the listener, ROS 2 is working correctly.

---

## 10. Common VMware-Specific Notes (Verified)

### Networking

* VMware must be set to **NAT networking**
* ROS 2 uses DDS and works **locally** without extra configuration

### Performance Tips

* Allocate **at least**:

  * 2 GB RAM (4+ GB recommended)
  * 2 CPU cores (4+ recommended)
* Enable **3D acceleration** in VMware settings

---

## 11. Known Issues & Fixes

### Problem: `ros2: command not found`

**Fix:**

```bash
source /opt/ros/humble/setup.bash
```

If that works, check `.bashrc`.

---

### Problem: Slow VM

**Fix:**

* Increase RAM/CPU in VMware
* Disable unnecessary Ubuntu startup apps

---

## 12. Summary (What You Should Have Now)

- Ubuntu 22.04
- ROS 2 Humble Desktop installed
- rosdep initialized
- ROS automatically sourced
- Verified with talker/listener
