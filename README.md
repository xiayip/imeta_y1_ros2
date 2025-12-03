# imeta_y1_ros2

Complete ROS2 stack for the **IMeta Y1** robotic arm platform. This metapackage provides robot description, hardware interfaces, controller configurations, and launch files for simulation and real-world deployment.

---

## 📦 Package Overview

This repository contains four interconnected packages:

| Package | Purpose |
|---------|---------|
| **imeta_y1** | Metapackage that groups all Y1 packages together |
| **imeta_y1_description** | URDF/Xacro robot model, meshes, RViz configs |
| **imeta_y1_bringup** | Launch files and controller configurations |
| **imeta_y1_hardware** | ros2_control hardware interface for real robot (Y1 SDK wrapper) |

---

## 🏗️ Repository Structure

```
imeta_y1_ros2/
├── imeta_y1/                    # Metapackage
│   ├── package.xml
│   └── CMakeLists.txt
│
├── imeta_y1_description/        # Robot model and visualization
│   ├── urdf/
│   │   ├── imeta_y1.urdf.xacro         # Main URDF entry point
│   │   └── imeta_y1/
│   │       ├── imeta_y1_macro.urdf.xacro          # Kinematic description
│   │       ├── imeta_y1_macro.ros2_control.xacro  # Hardware interface macro
│   │       └── include/
│   │           ├── mock.ros2_control.xacro        # Mock hardware
│   │           ├── real_world.ros2_control.xacro  # Real robot
│   │           ├── gazebo.ros2_control.xacro      # Gazebo (gz)
│   │           ├── gazebo_classic.ros2_control.xacro
│   │           ├── isaac_sim.ros2_control.xacro   # NVIDIA Isaac Sim
│   │           └── mujoco.ros2_control.xacro      # MuJoCo
│   ├── meshes/imeta_y1/         # STL visual/collision meshes
│   ├── rviz/imeta_y1.rviz       # RViz configuration
│   └── launch/
│       └── view_imeta_y1.launch.py  # Standalone visualization
│
├── imeta_y1_bringup/            # Launch and control configs
│   ├── launch/
│   │   ├── bringup.launch.py         # Generic bringup with mode selection
│   │   ├── mock.launch.py            # Quick mock hardware launch
│   │   ├── real_world.launch.py      # Real robot launch
│   │   ├── isaac_sim.launch.py       # Isaac Sim integration
│   │   ├── mujoco.launch.py          # MuJoCo integration
│   │   └── include/
│   │       └── controllers.launch.py # Controller spawner
│   └── config/
│       └── controllers.yaml          # Joint trajectory controller config
│
└── imeta_y1_hardware/           # Hardware interface plugin
    ├── src/imeta_y1_hardware.cpp
    ├── include/imeta_y1_hardware/
    ├── y1_sdk/                   # Vendor SDK (x64 and aarch64)
    └── README.md                 # Hardware-specific docs
```

---

## 🚀 Quick Start

### Prerequisites

- **ROS2 Humble** (Ubuntu 22.04)
- `ros-humble-ros2-control`
- `ros-humble-ros2-controllers`
- `ros-humble-xacro`
- `ros-humble-joint-state-publisher-gui` (for visualization)

### Build

```bash
# Navigate to workspace root
cd /path/to/your_workspace

# Build all Y1 packages
colcon build --symlink-install --packages-up-to imeta_y1

# Source the workspace
source install/setup.bash
```

---

## 🎮 Usage Examples

### 1. Visualize Robot in RViz (No Control)

Quickly view the robot model with joint sliders:

```bash
ros2 launch imeta_y1_description view_imeta_y1.launch.py
```

This launches:
- `robot_state_publisher` (publishes TF from URDF)
- `joint_state_publisher_gui` (manual joint control)
- RViz with pre-configured display

### 2. Mock Hardware Mode (Testing/Development)

Run with simulated hardware (no physics, instant command execution):

```bash
ros2 launch imeta_y1_bringup mock.launch.py
```

**What this does:**
- Loads the robot with `mock_hardware` plugin
- Starts `controller_manager`
- Spawns configured controllers from `config/controllers.yaml`
- Publishes joint states and accepts trajectory commands

**Test it:**
```bash
# List available controllers
ros2 control list_controllers

# Send a test trajectory (example - adjust joint names)
ros2 topic pub /joint_trajectory trajectory_msgs/msg/JointTrajectory ...
```

### 3. Real Robot Hardware

Connect to the physical Y1 arm via CAN bus:

```bash
ros2 launch imeta_y1_bringup real_world.launch.py
```

**Prerequisites:**
- CAN interface configured (e.g., `can0`)
- Y1 SDK libraries installed (see `imeta_y1_hardware/README.md`)
- Hardware parameters set in the ros2_control config

**CAN Setup Example:**

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
sudo ip link set can0 txqueuelen 1000
sudo ip link set up can0
```

**Hardware parameters** (configured in `real_world.ros2_control.xacro`):
- `can_interface`: CAN device name (e.g., `"can0"`)
- `arm_end_type`: End effector type code (integer)
- `enable_arm`: `"true"` or `"false"`

### 4. Isaac Sim Integration

Run with NVIDIA Isaac Sim physics simulation:

```bash
ros2 launch imeta_y1_bringup isaac_sim.launch.py
```

**Note:** Requires Isaac Sim to be running and configured separately.

### 5. MuJoCo Simulation

```bash
ros2 launch imeta_y1_bringup mujoco.launch.py
```

### 6. Generic Bringup (Select Mode)

The `bringup.launch.py` allows mode selection via arguments:

```bash
# Mock hardware
ros2 launch imeta_y1_bringup bringup.launch.py use_mock_hardware:=true

# Real robot
ros2 launch imeta_y1_bringup bringup.launch.py real_world:=true

# Isaac Sim
ros2 launch imeta_y1_bringup bringup.launch.py sim_isaac_sim:=true
```

---

## 🎛️ Controller Configuration

Controllers are defined in `imeta_y1_bringup/config/controllers.yaml`. The default setup includes:

- **`joint_state_broadcaster`** — Publishes `/joint_states`
- **`joint_trajectory_controller`** — Accepts trajectory goals

### Controller Interface

To send commands to the robot:

```bash
# Via topic (direct)
ros2 topic pub --once /joint_trajectory trajectory_msgs/msg/JointTrajectory \
"header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: ''
joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
points:
  - positions: [0.6, -0.6, 0.6, 0.5, 0.4, 0.0]
    time_from_start:
      sec: 2
      nanosec: 0"
```

### Add Custom Controllers

1. Edit `config/controllers.yaml` to add your controller
2. Update `launch/include/controllers.launch.py` to spawn it
3. Rebuild and relaunch

---

## 🔧 Hardware Interface Details

The `imeta_y1_hardware` package provides the ros2_control `SystemInterface` that communicates with the real robot via the Y1 SDK.

### Key Features

- CAN bus communication
- Joint state feedback (position, velocity, effort)
- Command execution (position, velocity, effort)
- Safe initialization and error handling

### Configuration

Hardware parameters are passed through the ros2_control URDF tag. Example from `real_world.ros2_control.xacro`:

```xml
<hardware>
  <plugin>imeta_y1_hardware/IMetaY1HW</plugin>
  <param name="can_interface">can0</param>
  <!-- 0: only arm, 1: gripper_T, 2: gripper_G, 3: gripper_GT -->
  <param name="arm_end_type">0</param>
  <param name="enable_arm">true</param>
</hardware>
```

See `imeta_y1_hardware/README.md` for detailed setup instructions.

---

## 📚 Additional Resources

### Related Packages

- [ros2_control](https://control.ros.org/master/index.html) — ROS2 control framework
- [ros2_controllers](https://control.ros.org/master/doc/ros2_controllers/doc/controllers_index.html) — Standard controllers

### Useful Commands

```bash
# View robot model
ros2 launch imeta_y1_description view_imeta_y1.launch.py

# List all controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# Check TF tree
ros2 run tf2_tools view_frames

```

---

## 🔍 Quick Reference

| Task | Command |
|------|---------|
| View robot | `ros2 launch imeta_y1_description view_imeta_y1.launch.py` |
| Mock hardware | `ros2 launch imeta_y1_bringup mock.launch.py` |
| Real robot | `ros2 launch imeta_y1_bringup real_world.launch.py` |
| Isaac Sim | `ros2 launch imeta_y1_bringup isaac_sim.launch.py` |
| List controllers | `ros2 control list_controllers` |
| Monitor joints | `ros2 topic echo /joint_states` |
