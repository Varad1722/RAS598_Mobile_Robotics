# RAS 598 – Mobile Robotics — Lab 2: TF, TurtleBot4 Simulation, and Sensor Frame Analysis

**Course:** RAS 598 – Mobile Robotics  
**Lab:** Lab 2  
**Platform:** ROS 2 Jazzy  
**Simulation:** Gazebo (TurtleBot4)

## Objective
The objective of this lab is to gain hands-on experience with ROS 2 coordinate transforms (TF2), simulation using Gazebo, and frame analysis on the TurtleBot4 platform. The lab focuses on understanding how spatial relationships between robot components are represented using TF, how these transforms are visualized, and how sensor offsets relative to the robot’s center frame can be computed using ROS 2 tools.

## Software and Tools Used
- ROS 2 Jazzy  
- Gazebo Simulator  
- TurtleBot4 Simulation  
- TF2 (`tf2_ros`, `tf2_tools`)  
- `teleop_twist_keyboard`  
- Python (`rclpy`)  
- GitHub for version control  

## Task 1: TF with Turtlesim
### Description
In this task, a basic TF2 pipeline was implemented using the turtlesim simulator. Two turtles were spawned: one turtle was manually controlled using keyboard teleoperation, and the second turtle followed the first using TF transforms.

### Key Components
- Dynamic TF broadcaster for each turtle  
- TF listener to compute relative transforms  
- Keyboard teleoperation for motion control  

### Commands Used
- `ros2 run turtlesim turtlesim_node`  
- `ros2 run turtlesim turtle_teleop_key`  
- `ros2 run learning_tf2_py turtle_tf2_broadcaster --ros-args -p turtlename:=turtle1`  
- `ros2 run learning_tf2_py turtle_tf2_broadcaster --ros-args -p turtlename:=turtle2`  
- `ros2 run learning_tf2_py turtle_tf2_listener --ros-args -p target_frame:=turtle1`  

### Outcome
- Successful broadcasting of transforms between turtles  
- The follower turtle tracked the leader turtle using TF-based relative positioning  
- TF tree correctness verified using `tf2_tools view_frames`
### Video: https://youtu.be/RvHSjfnX1Yo 

## Task 2: TurtleBot4 Gazebo Simulation
### Description
The TurtleBot4 Gazebo simulation was launched using the provided `ras598_sim` package. The robot was initialized, undocked, and prepared for user control.

### Key Components
- TurtleBot4 simulation environment  
- Docking and undocking control  
- Stable Gazebo rendering configuration  

### Commands Used
- `git clone https://github.com/ras-mobile-robotics/ras598_sim.git`  
- `rosdep install -i --from-path src --rosdistro jazzy -y`  
- `colcon build`  
- `ros2 launch ras598_sim turtlebot4_gz.launch.py`  
- `ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"`  

### Outcome
- TurtleBot4 simulation launched successfully  
- Robot undocked and ready to receive velocity commands  
- Gazebo visualization rendered correctly without instability  

## Task 3: Keyboard Teleoperation of TurtleBot4
### Description
Keyboard teleoperation was enabled to manually control the TurtleBot4 within the Gazebo simulation using velocity commands.

### Key Components
- Command velocity interface  
- `TwistStamped` message handling  
- Keyboard-based teleoperation  

### Commands Used
- `ros2 topic info /cmd_vel`  
- `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true -r cmd_vel:=/cmd_vel`  

### Outcome
- TurtleBot4 responded correctly to keyboard input  
- Linear and angular motions executed as expected
### Video: https://youtu.be/aIy_oS6VGCI 

## Task 4: TF Tree Visualization
### Description
The complete TF tree of the TurtleBot4 system was visualized to analyze the hierarchical relationships between robot and sensor frames.

### Key Components
- TF topic verification  
- Frame hierarchy visualization  
- Identification of static and dynamic transforms  

### Commands Used
- `ros2 topic list | grep tf`  
- `ros2 run tf2_tools view_frames`  

### Outcome
- TF tree successfully generated and visualized  
- Parent–child relationships between frames verified  
- Sensor frames, including the OAK-D Lite camera, identified  

## Task 5: Camera Offset from Robot Center of Mass
### Description
The spatial offset of the OAK-D Lite camera relative to the robot’s base frame (`base_link`) was computed using TF2 tools.

### Key Components
- Identification of the camera mounting frame  
- Live TF transform inspection  
- Extraction of translation components  

### Commands Used
- `ros2 run tf2_ros tf2_echo base_link oakd_link`  

### Outcome
- Camera offset relative to `base_link` successfully computed  
- Translation values (x, y, z) obtained in meters  
- Camera placement confirmed to be offset from the robot’s center of mass  

## Conclusion
This lab provided comprehensive exposure to TF2, Gazebo-based simulation, and sensor frame analysis in ROS 2. The tasks reinforced the importance of coordinate transforms in mobile robot perception, control, and navigation, forming a strong foundation for future work in autonomous robotics.

