# Panda MoveIt Configuration Project

A comprehensive ROS 2 MoveIt configuration package for the Franka Emika Panda robot arm, featuring simulation, motion planning, and web-based control interfaces.

## 🤖 Project Overview

This project provides a complete MoveIt configuration for the Panda robot arm with:
- **Gazebo Simulation Integration**: Full physics simulation with ros2_control
- **Motion Planning**: OMPL, CHOMP, and Pilz Industrial Motion Planner support
- **Custom FK/IK Nodes**: Forward and Inverse Kinematics implementations
- **Web Interface**: Modern web-based control panel for real-time robot control
- **Multiple Launch Configurations**: Demo, simulation, and various robot setups

## 📁 Project Structure

```
src/moveit_resources_panda_moveit_config/
├── CMakeLists.txt                 # Build configuration
├── package.xml                    # Package metadata and dependencies
├── config/                        # Configuration files
│   ├── chomp_planning.yaml       # CHOMP planner configuration
│   ├── gripper_moveit_controllers.yaml # Gripper controller config
│   ├── hand.xacro                # Hand SRDF definitions
│   ├── initial_positions.yaml    # Default joint positions
│   ├── joint_limits.yaml         # Joint velocity/acceleration limits
│   ├── kinematics.yaml           # Kinematics solver configuration
│   ├── lerp_planning.yaml        # LERP planner configuration
│   ├── moveit_controllers.yaml   # MoveIt controller configuration
│   ├── ompl_planning.yaml        # OMPL planner configurations
│   ├── panda.ros2_control.xacro  # ROS2 control interface
│   ├── panda.srdf                # Semantic robot description
│   ├── panda.urdf                # Robot description (generated)
│   ├── panda.urdf.xacro          # Robot description (xacro)
│   ├── panda.xacro               # Complete robot xacro
│   ├── panda_*.xacro             # Component xacro files
│   ├── pilz_*.yaml               # Pilz planner configurations
│   ├── ros2_controllers.yaml     # ROS2 controller configuration
│   ├── sensors_*.yaml            # Sensor configurations
│   └── trajopt_planning.yaml     # TrajOpt planner configuration
├── launch/                        # Launch files
│   ├── demo.launch.py            # ⭐ Main MoveIt demo (no simulation)
│   ├── full_simulation.launch.py # Complete Gazebo + MoveIt setup
│   ├── gazebo.launch.py          # Basic Gazebo launch
│   ├── gazebo_simulation.launch.py # ⭐ Main Gazebo simulation
│   ├── moveit.rviz               # RViz configuration (full)
│   ├── moveit_empty.rviz         # RViz configuration (minimal)
│   └── moveit_rviz.launch.py     # RViz-only launch
├── scripts/                       # Custom nodes
│   ├── panda_fk_node.cpp         # Forward Kinematics node
│   └── panda_ik_node.cpp         # Inverse Kinematics node
├── web_backend/                   # Web server backend
│   └── backend.py                # ⭐ Flask API server
└── web_frontend/                  # Web interface
    └── panda_ik_frontend.html    # ⭐ Interactive control panel
```

## 🔧 Features

### Core MoveIt Features
- **Motion Planning**: Support for OMPL, CHOMP, and Pilz Industrial Motion Planner
- **Trajectory Execution**: Full trajectory planning and execution capabilities
- **Collision Detection**: Real-time collision checking and avoidance
- **Scene Monitoring**: Dynamic planning scene updates

### Simulation Features
- **Gazebo Integration**: Full physics simulation with proper inertial properties
- **ros2_control**: Hardware abstraction layer for seamless sim-to-real transition
- **Controller Management**: Joint state broadcaster and trajectory controllers
- **Visualization**: RViz integration with motion planning plugin

### Custom Implementations
- **Forward Kinematics**: Custom FK implementation with DH parameters
- **Inverse Kinematics**: Newton-Raphson based IK solver with joint limits
- **Parameter Control**: Dynamic parameter updates via ROS 2 parameters
- **Web Interface**: Real-time browser-based robot control

## 🚀 Quick Start Guide

### Primary Launch Files (Main Usage)

1. **MoveIt Demo (No Simulation)**
   ```bash
   ros2 launch moveit_resources_panda_moveit_config demo.launch.py
   ```

2. **Gazebo Simulation**
   ```bash
   ros2 launch moveit_resources_panda_moveit_config gazebo_simulation.launch.py
   ```

3. **Web Interface Control**
   ```bash
   # Terminal 1: Start FK node
   ros2 run moveit_resources_panda_moveit_config panda_fk_node
   
   # Terminal 2: Start web backend
   cd src/moveit_resources_panda_moveit_config/web_backend
   python3 backend.py
   
   # Terminal 3: Open web interface
   cd src/moveit_resources_panda_moveit_config/web_frontend
   python3 -m http.server
   # Open web_frontend/panda_ik_frontend.html in browser
   ```

4. **Foxglove Bridge Integration**
   ```bash
   # For visualization in Foxglove Studio
   ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
   # Then connect Foxglove Studio to ws://localhost:8765
   ```

## 🚀 Installation & Setup
```bash
# ROS 2 Humble (recommended)
sudo apt update
sudo apt install ros-humble-desktop

# MoveIt 2
sudo apt install ros-humble-moveit

# Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# Additional dependencies
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-joint-trajectory-controller
sudo apt install ros-humble-joint-state-broadcaster

# Additional dependencies for Foxglove integration
sudo apt install ros-humble-foxglove-bridge

# Python dependencies for web interface
pip3 install flask flask-cors
```

### Build Instructions
```bash
# Create workspace
mkdir -p ~/panda_ws/src
cd ~/panda_ws/src

# Clone your project (adjust path as needed)
# cp -r /path/to/moveit_resources_panda_moveit_config .

# Build
cd ~/panda_ws
colcon build --packages-select moveit_resources_panda_moveit_config

# Source the workspace
source ~/panda_ws/install/setup.bash
```

## 🎯 Usage Examples

### 1. Basic MoveIt Demo (Recommended Start)
Launch MoveIt with RViz for motion planning:
```bash
ros2 launch moveit_resources_panda_moveit_config demo.launch.py
```
**Features**: Motion planning, RViz visualization, no physics simulation

### 2. Gazebo Physics Simulation (Main Simulation)
Complete simulation with physics and controllers:
```bash
ros2 launch moveit_resources_panda_moveit_config gazebo_simulation.launch.py
```
**Features**: Gazebo physics, ROS2 controllers, real-time simulation

### 3. Foxglove Studio Visualization
Enhanced visualization and debugging:
```bash
# Terminal 1: Launch any simulation
ros2 launch moveit_resources_panda_moveit_config gazebo_simulation.launch.py

# Terminal 2: Start Foxglove bridge
ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765

# Terminal 3: Open Foxglove Studio and connect to ws://localhost:8765
```
**Features**: Advanced 3D visualization, real-time plotting, data recording

### 4. Custom FK/IK Nodes

#### Forward Kinematics Node
```bash
# Launch the FK node
ros2 run moveit_resources_panda_moveit_config panda_fk_node

# Set joint angles dynamically
ros2 param set /panda_fk_node fk_joint1 45.0
ros2 param set /panda_fk_node fk_joint2 -30.0
```

#### Inverse Kinematics Node
```bash
# Launch the IK node
ros2 run moveit_resources_panda_moveit_config panda_ik_node

# Send target position
ros2 topic pub /target_position geometry_msgs/msg/Point "{x: 0.5, y: 0.2, z: 0.4}"
```

### 5. Web Interface Control (Interactive)

#### Start the Backend Server
```bash
cd ~/panda_ws/src/moveit_resources_panda_moveit_config/web_backend
python3 backend.py
```

#### Access the Web Interface
1. Start the FK node: `ros2 run moveit_resources_panda_moveit_config panda_fk_node`
2. Open `web_frontend/panda_ik_frontend.html` in your browser
3. Use the interactive controls to move the robot

**Web Interface Features:**
- 🎯 Real-time joint angle control
- 📊 Joint limit visualization
- 🏠 Preset positions (Ready, Extended, Transport)
- ⚡ Live parameter updates
- 🤖 Modern, responsive design

## ⚙️ Configuration

## ⚙️ Configuration Files Detailed

### Planning Configurations

#### OMPL Planner (`ompl_planning.yaml`)
- **Available Planners**: RRT, RRTConnect, RRTstar, PRM, EST, KPIECE, FMT, BFMT, and more
- **Key Parameters**:
  - `range`: Maximum motion added to tree (0.0 = auto-setup)
  - `goal_bias`: Probability of selecting goal when close (default: 0.05)
  - **Supported Groups**: `panda_arm`, `panda_arm_hand`, `hand`

#### CHOMP Planner (`chomp_planning.yaml`)
- **Optimization-based** trajectory planning
- **Key Parameters**:
  - `max_iterations`: 200
  - `smoothness_cost_weight`: 0.1
  - `obstacle_cost_weight`: 1.0
  - `collision_clearance`: 0.2m
  - `learning_rate`: 0.01

#### Pilz Industrial Motion Planner (`pilz_industrial_motion_planner_planning.yaml`)
- **Industrial-grade** motion planning
- **Capabilities**: MoveGroupSequenceAction/Service
- **Cartesian Limits** (`pilz_cartesian_limits.yaml`):
  - Max translation velocity: 1.0 m/s
  - Max translation acceleration: 2.25 m/s²
  - Max rotation velocity: 1.57 rad/s

#### TrajOpt Planner (`trajopt_planning.yaml`)
- **Trajectory optimization** using sequential convex optimization
- **Key Parameters**:
  - `n_steps`: 20 (trajectory discretization)
  - `max_iter`: 100
  - `trust_box_size`: 0.1
  - `convex_solver`: AUTO_SOLVER

### Robot Configuration

#### Joint Limits (`joint_limits.yaml`)
**Velocity Limits** (rad/s):
- Joints 1-4: 2.175
- Joints 5-7: 2.61
- Finger joints: 0.1

**Acceleration Limits** (rad/s²):
- Joint 1: 3.75
- Joint 2: 1.875
- Joint 3: 2.5
- Joint 4: 3.125
- Joints 5-7: 3.75-5.0

#### Initial Positions (`initial_positions.yaml`)
Default "ready" position (radians):
```yaml
panda_joint1: 0.0     # 0°
panda_joint2: -0.785  # -45°
panda_joint3: 0.0     # 0°
panda_joint4: -2.356  # -135°
panda_joint5: 0.0     # 0°
panda_joint6: 1.571   # 90°
panda_joint7: 0.785   # 45°
```

#### Semantic Robot Description (`panda.srdf`)
**Planning Groups**:
- `panda_arm`: 7-DOF arm (panda_link0 → panda_link8)
- `hand`: 2-DOF gripper (parallel fingers)
- `panda_arm_hand`: Combined arm + hand

**Named States**:
- **ready**: Standard operational pose
- **extended**: Fully extended arm
- **transport**: Compact transport position
- **open/close**: Gripper states (0.035m / 0.0m)

**End Effector**: Hand group attached to panda_link8

### Control Configuration

#### ROS2 Controllers (`ros2_controllers.yaml`)
- **Update Rate**: 100 Hz
- **Arm Controller**: `JointTrajectoryController`
  - Command interface: position
  - State interfaces: position, velocity
- **Hand Controller**: `GripperActionController`
  - Parallel gripper control
- **Joint State Broadcaster**: Publishes `/joint_states`

#### MoveIt Controllers (`moveit_controllers.yaml` & `gripper_moveit_controllers.yaml`)
- **Trajectory Execution**:
  - Allowed duration scaling: 1.2
  - Goal duration margin: 0.5s
  - Start tolerance: 0.01
- **Controller Types**:
  - `FollowJointTrajectory`: For arm motion
  - `GripperCommand`: For hand control

### Kinematics Configuration

#### Kinematics Solver (`kinematics.yaml`)
- **Solver**: KDL (Kinematics and Dynamics Library)
- **Search Resolution**: 0.005
- **Timeout**: 0.05 seconds
- **Group**: panda_arm (7-DOF)

### URDF/Xacro Structure

#### Main Robot Description (`panda.urdf.xacro`)
- **Parameterized** robot description
- **Arguments**:
  - `initial_positions_file`: Default joint positions
  - `ros2_control_hardware_type`: Hardware interface type
- **Includes**: Panda URDF + ROS2 control definitions

#### ROS2 Control Integration (`panda.ros2_control.xacro`)
- **Hardware Plugin**: `gazebo_ros2_control/GazeboSystem`
- **Joint Interfaces**: Position command + position/velocity state
- **Initial Values**: Loaded from YAML configuration

### Sensor Configuration

#### Kinect Integration
- **Depth Image** (`sensors_kinect_depthmap.yaml`):
  - Topic: `/camera/depth_registered/image_raw`
  - Range: 0.3-5.0m
  - Update rate: 1.0 Hz
- **Point Cloud** (`sensors_kinect_pointcloud.yaml`):
  - Topic: `/camera/depth_registered/points`
  - Max range: 5.0m
  - Padding: 0.1m

### Collision Configuration
**Disabled Collision Pairs** (optimized for performance):
- Adjacent links (parent-child relationships)
- Links that never collide in valid configurations
- Hand-arm collision exclusions for manipulation tasks

## 🔍 Troubleshooting

### Common Issues

1. **Controllers not loading in Gazebo**
   ```bash
   # Check controller manager status
   ros2 control list_controllers
   
   # Manually load controllers if needed
   ros2 control load_controller joint_state_broadcaster
   ros2 control load_controller panda_arm_controller
   ros2 control load_controller panda_hand_controller
   
   # Check controller configuration
   ros2 param describe /controller_manager
   ```

2. **Gazebo simulation timing issues**
   ```bash
   # Ensure proper timing
   ros2 param set /use_sim_time true
   
   # Check joint states
   ros2 topic echo /joint_states
   
   # Monitor simulation time
   ros2 topic echo /clock
   ```

3. **Web interface connection problems**
   - Ensure Flask backend is running on port 5000
   - Check CORS configuration if accessing from different domain
   - Verify ROS 2 nodes are active: `ros2 node list`
   - Test parameter setting: `ros2 param set /panda_fk_node fk_joint1 45.0`

4. **Foxglove Bridge issues**
   ```bash
   # Check bridge status
   ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8765
   
   # Test WebSocket connection
   ros2 topic list  # Should show all topics
   
   # Alternative port if 8765 is busy
   ros2 run foxglove_bridge foxglove_bridge --ros-args -p port:=8766
   ```

5. **MoveIt planning failures**
   ```bash
   # Check planning scene
   ros2 topic echo /monitored_planning_scene
   
   # Verify kinematics solver
   ros2 param get /move_group kinematics_solver
   
   # Test different planners
   # In RViz Motion Planning: Planning → Algorithm → RRTConnect/OMPL
   ```

6. **URDF/SRDF loading errors**
   ```bash
   # Validate URDF
   check_urdf src/moveit_resources_panda_moveit_config/config/panda.urdf
   
   # Test xacro processing
   xacro src/moveit_resources_panda_moveit_config/config/panda.urdf.xacro
   
   # Check for missing dependencies
   rosdep check moveit_resources_panda_moveit_config
   ```

### Debug Commands
```bash
# Check active nodes
ros2 node list

# Monitor joint states
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo world panda_link0

# Controller status
ros2 control list_controllers
ros2 control list_hardware_interfaces

# MoveIt status
ros2 topic list | grep move_group
ros2 service list | grep move_group

# Web backend testing
curl -X POST http://localhost:5000/set_param \
  -H "Content-Type: application/json" \
  -d '{"name": "fk_joint1", "value": 30.0}'
```

### Performance Optimization
```bash
# Adjust Gazebo real-time factor
ros2 param set /gazebo real_time_factor 1.0

# Reduce planning time for faster responses
ros2 param set /move_group/ompl_planning_time 2.0

# Optimize RViz display frequency
# In RViz: Global Options → Frame Rate → 30
```

## 🧩 Technical Details

### Forward Kinematics Implementation
- Uses Denavit-Hartenberg (DH) parameters
- Supports dynamic parameter updates
- Publishes joint trajectories to controller
- Real-time end-effector position calculation

### Inverse Kinematics Solver
- Newton-Raphson iterative method
- Jacobian-based approach with damping
- Joint limit enforcement
- Configurable tolerance and iteration limits
- Singular configuration detection

### Web Architecture
- **Backend**: Flask REST API for parameter control
- **Frontend**: HTML interface
- **Communication**: HTTP POST requests for parameter updates
- **Security**: Parameter validation and type checking

## 📊 Dependencies

### Build Dependencies
- `ament_cmake`
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `trajectory_msgs`
- `Eigen3`

### Runtime Dependencies
- `moveit_resources_panda_description`
- `joint_state_publisher`
- `robot_state_publisher`
- `xacro`
- `gazebo_ros`
- `controller_manager`

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## 📝 License

This project is licensed under the BSD License. See the package.xml for details.

## 🙏 Acknowledgments

- MoveIt community for the excellent motion planning framework
- Franka Emika for the Panda robot specifications
- ROS 2 community for the robust robotics middleware

---