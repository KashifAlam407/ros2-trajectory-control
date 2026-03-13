# Path Smoothing and Trajectory Control for Differential Drive Robot (ROS2)

## Project Overview

This project implements a **complete trajectory generation and tracking pipeline** for a differential drive robot using **ROS2**. The system takes a set of **discrete 2D waypoints**, generates a **smooth path**, converts it into a **time-parameterized trajectory**, and then uses a **controller to make the robot follow the trajectory**.

The implementation is tested in simulation using **Turtlebot3 in Gazebo** and visualized using **RViz**.

The system is designed in a **modular ROS2 architecture**, where each node performs a specific task. This approach improves readability, debugging, and scalability.

---

# Code Repository

The project is implemented using **Python and ROS2** and runs on a **Turtlebot3 differential drive robot simulation**.

The system is organized into multiple ROS2 nodes with clear responsibilities.

## Folder Structure

```
trajectory_ws/
│
├── src/
│   └── trajectory_control/
│
│       ├── launch/
│       │   └── trajectory_launch.py
│       │
│       ├── trajectory_control/
│       │   ├── __init__.py
│       │   ├── waypoint_publisher.py
│       │   ├── path_smoother.py
│       │   ├── trajectory_generator.py
│       │   └── controller.py
│       │
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
│
└── README.md
```

---

# System Architecture

The navigation pipeline is structured as follows:

```
Waypoints → Smooth Path → Time-Stamped Trajectory → Controller → Robot Motion
```

Topic flow:

```
/waypoints
     ↓
/smooth_path
     ↓
/trajectory
     ↓
/cmd_vel
```

Each component is implemented as a **separate ROS2 node** to maintain modularity.

---

# Node Descriptions

## 1. waypoint_publisher.py

**Purpose**

Publishes a list of predefined **2D waypoints** that represent a coarse path for the robot.

**Input**

None

**Output Topic**

```
/waypoints
```

**Message Type**

```
geometry_msgs/PoseArray
```

The node periodically publishes waypoint coordinates that define the path the robot should follow.

Example waypoints:

```
[-2.0, -0.5]
[-1.0, -0.5]
[-0.5, -0.5]
[0.0, -0.5]
[1.0, 0.5]
[2.0, 0.0]
```

---

## 2. path_smoother.py

**Purpose**

Convert the discrete waypoints into a **smooth continuous path** that the robot can follow.

**Algorithm**

Cubic Spline Interpolation

Libraries used:

```
numpy
scipy.interpolate.CubicSpline
```

**Process**

1. Extract x and y coordinates from waypoints
2. Fit cubic spline curves
3. Sample the spline into 100 points
4. Publish a smooth path

**Output Topic**

```
/smooth_path
```

**Message Type**

```
nav_msgs/Path
```

This produces a smooth curve suitable for robot motion.

---

## 3. trajectory_generator.py

**Purpose**

Convert the smooth path into a **time-parameterized trajectory**.

A path contains only spatial information:

```
(x, y)
```

A trajectory adds time:

```
(x, y, t)
```

**Algorithm**

Time is calculated using:

```
time = distance / velocity
```

Steps:

1. Compute distance between consecutive points
2. Assume constant robot velocity
3. Compute travel time
4. Assign timestamps to each pose

**Output Topic**

```
/trajectory
```

**Message Type**

```
nav_msgs/Path
```

Each trajectory point contains:

```
position + timestamp
```

---

## 4. controller.py

**Purpose**

Control the robot so it follows the generated trajectory.

**Subscribed Topics**

```
/trajectory
/odom
```

**Published Topic**

```
/cmd_vel
```

**Message Type**

```
geometry_msgs/Twist
```

The controller performs the following steps:

1. Read the robot pose from odometry
2. Select a lookahead point on the trajectory
3. Compute heading error
4. Generate velocity commands
5. Stop the robot when the final goal is reached

This approach ensures smooth trajectory tracking.

---

# Setup and Execution Instructions

## 1. Create Workspace

```
mkdir -p ~/trajectory_ws/src
cd ~/trajectory_ws/src
```

Copy the `trajectory_control` package into the `src` folder.

---

## 2. Build Workspace

```
cd ~/trajectory_ws
colcon build
```

---

## 3. Source Workspace

```
source install/setup.bash
```

---

## 4. Launch Turtlebot3 Simulation

```
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

## 5. Run the Trajectory System

```
ros2 launch trajectory_control trajectory_launch.py
```

This launch file starts all nodes:

- waypoint publisher
- path smoother
- trajectory generator
- controller

---

# Visualization

Open RViz and add the following displays:

```
RobotModel
TF
Odometry
Path → /smooth_path
Path → /trajectory
```

This allows visualization of the robot and the generated trajectory.

---

# Design Choices and Architectural Decisions

### Modular Node Architecture

The system is divided into independent nodes, each responsible for a specific task:

- waypoint generation
- path smoothing
- trajectory generation
- trajectory tracking

This modular design makes the system easier to debug and extend.

---

### Cubic Spline Path Smoothing

Cubic splines were chosen because they produce:

- smooth curvature
- continuous derivatives
- realistic robot paths

This avoids sharp turns that differential drive robots cannot follow.

---

### Time-Parameterized Trajectory

Adding timestamps enables predictable robot motion and synchronization between trajectory points and robot control.

---

### Lookahead Controller

The controller uses a lookahead strategy to follow the trajectory smoothly.

Advantages:

- stable tracking
- smooth motion
- simple implementation

---

# Extending the System to a Real Robot

To deploy the system on a real robot, the following changes would be required:

1. Replace simulated `/odom` with real odometry from wheel encoders or localization.
2. Connect `/cmd_vel` to the robot motor controller.
3. Add velocity and safety constraints.

Sensors typically used on a real robot:

- Wheel encoders
- IMU
- LiDAR
- Cameras

These sensors provide accurate localization and obstacle detection.

---

# AI Tools Used

AI tools were used during development to assist with:

- understanding ROS2 architecture
- debugging trajectory controller behavior
- explaining mathematical concepts
- improving documentation structure

The final implementation and testing were performed manually in simulation.

---

# Extra Credit: Extending the System for Obstacle Avoidance

Obstacle avoidance can be added by integrating sensor data from LiDAR or depth cameras.

Steps:

1. Subscribe to LiDAR data

```
/scan
```

2. Detect obstacles along the trajectory.

3. Modify trajectory if an obstacle is detected.

Possible algorithms:

- Dynamic Window Approach (DWA)
- Model Predictive Control
- Potential Fields

These methods allow the robot to adjust its motion while avoiding obstacles.

---

# Demonstration Video

The demonstration video shows:

1. Waypoint publishing
2. Path smoothing
3. Trajectory generation
4. Robot following the trajectory
5. Robot stopping at the goal

The visualization includes RViz and Gazebo simulation.

---

# Conclusion

This project demonstrates a complete pipeline for **path smoothing and trajectory tracking using ROS2**.

Key features include:

- cubic spline path smoothing
- time-parameterized trajectory generation
- trajectory tracking controller
- modular ROS2 architecture

The robot successfully follows a smooth trajectory and stops at the final goal.
