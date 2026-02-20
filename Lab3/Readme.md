
# 🤖 RAS 598 – Mobile Robotics — Lab 3: Physical TurtleBot4 Setup, RViz Visualization, and Motion Control

**Course:** RAS 598 – Mobile Robotics  
**Lab:** Lab 3  
**Platform:** ROS 2 Jazzy  
**Hardware:** TurtleBot4 (Physical Robot)

---

## 🎯 Objective

The objective of this lab is to transition from simulation to real hardware by configuring a Virtual Machine (VM) to communicate with a physical TurtleBot4 using a FastDDS Discovery Server. The lab focuses on verifying ROS 2 communication, visualizing real-time sensor data in RViz2, inspecting velocity commands, and implementing a custom Python node to drive the robot in a geometric path using odometry feedback.

---

## 🧰 Software and Tools Used

- ROS 2 Jazzy
- TurtleBot4 (Physical Robot)
- FastDDS Discovery Server
- RViz2
- teleop_twist_keyboard
- TwistStamped message type
- nav_msgs/Odometry
- SSH
- Ubuntu 24.04 VM (Bridged Network Mode)
- GitHub for version control

---

## 🔌 Task 1: Robot Setup and Power Management

### Description
The TurtleBot4 hardware was initialized and safely managed to prevent system corruption. Proper startup and shutdown procedures were followed to ensure stable communication and hardware protection.

### Key Components
- Charging dock initialization
- Raspberry Pi and Create® 3 synchronization
- Graceful shutdown procedure

### Commands Used
```bash
sudo shutdown -h now
```

### Outcome
- Robot initialized successfully
- LiDAR and onboard systems booted correctly
- Safe shutdown completed without hardware issues

---

## 🌐 Task 2: ROS2 Environment and Network Configuration

### Description
The VM was configured to communicate exclusively with the assigned robot using ROS 2 Domain IDs and a FastDDS Discovery Server. Bridged network mode ensured direct communication over the lab WiFi network.

### Key Components
- Discovery Server configuration
- ROS 2 environment setup script
- Bridged networking verification

### Commands Used
```bash
cd ~/vm_setup
git pull
./setup_robot_env.sh
set-ros-env robot
sudo reboot -h now
ip route get 8.8.8.8
```

### Outcome
- VM successfully connected to the robot
- ROS 2 nodes and topics visible
- DDS discovery functioning correctly

---

## 🔐 Task 3: Remote Access and Hardware Verification

### Description
SSH was used to access the TurtleBot4 and verify onboard nodes and sensors.

### Key Components
- Network connectivity verification
- ROS 2 node inspection
- Topic monitoring

### Commands Used
```bash
ping turtlebot
ssh ubuntu@turtlebot
ros2 node list
ros2 topic echo /robot_<XX>/battery_state
```

### Outcome
- SSH connection established successfully
- Robot nodes detected (`/motion_control`, `/oakd`)
- Battery state confirmed active ROS communication

---

## 👁️ Task 4: Sensor Visualization in RViz2

### Description
RViz2 was launched locally on the VM to visualize real-time sensor data streamed from the physical TurtleBot4.

### Key Components
- Fixed frame configuration
- LiDAR scan visualization
- TF tree visualization
- OAK-D camera feed

### Commands Used
```bash
run-lab-rviz
```

### Configuration

| Display | Topic |
|--------|-------|
| Fixed Frame | `rplidar_link` |
| LaserScan | `/robot_<XX>/scan` |
| TF | *(display only)* |
| Image | `/robot_<XX>/robot_11/oakd/rgb/preview/image_raw` |

### Outcome
- Live LiDAR data visualized correctly
- TF hierarchy displayed
- Camera feed rendered in real time

---

## 🎮 Task 5: Manual Control and Topic Inspection

### Description
Keyboard teleoperation was used to manually control the robot while inspecting velocity commands published to `/cmd_vel`.

### Key Components
- Teleoperation node
- Velocity topic monitoring
- Linear and angular motion analysis

### Commands Used
```bash
run-lab-teleop
ros2 topic echo /robot_<XX>/cmd_vel
```

### Outcome

| Motion Type | Field |
|------------|-------|
| Straight motion | `linear.x` non-zero |
| Rotation | `angular.z` non-zero |
| Curved motion | Both non-zero |

---

## 🤖 Task 6: Motion Node Implementation

### Description
A custom ROS 2 Python node (`robot_drive.py`) was developed to autonomously navigate the TurtleBot4 in a hexagonal path using odometry feedback instead of time-based motion control.

### Key Components
- Odometry subscription (`/robot_11/odom`)
- Distance tracking using Euclidean calculation
- Yaw extraction from quaternion
- Angle normalization
- Phase-based forward and rotation control

### Motion Parameters

| Parameter | Value |
|-----------|-------|
| Side Length | 1.0 m |
| Number of Sides | 6 |
| Turn Angle | 60° |
| Linear Velocity | 0.2 m/s |
| Angular Velocity | 0.5 rad/s |
| Distance Tolerance | 0.02 m |
| Angle Tolerance | 2° |

### Outcome
- Robot successfully completed a closed hexagonal path
- Straight segments maintained consistent length
- Rotations achieved approximately 60° with minimal drift
- Odometry-based control improved precision compared to time-based motion

---

## 📊 Results

- ✅ Successful communication between VM and physical TurtleBot4
- ✅ Stable real-time visualization of LiDAR and camera data
- ✅ Accurate execution of a six-sided geometric trajectory
- ✅ Minimal cumulative drift observed
- ✅ No network instability during execution

