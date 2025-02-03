# Bumperbot Robot

## Overview
Bumperbot is a 3D differential drive robot simulated in Gazebo using ROS 2. It consists of multiple packages that enable simulation, control, and localization.

---

## Bumperbot Description
The `bumperbot_description` package provides the necessary URDF and launch files to spawn the robot in an empty Gazebo world.

### Launching the Robot
To launch the robot in an empty Gazebo world, run:
```bash
ros2 launch bumperbot_description gazebo.launch.py
```

![Bumperbot in Gazebo](path/to/your/image.png)

### Sensors and Control
- The robot is equipped with an **IMU** for orientation sensing.
- Motion control is handled using `gazebo_ros2_control/GazeboSystem`.

---

## Bumperbot Controller
The `bumperbot_controller` package contains both **Python** and **C++** implementations for controlling the robot.

### Controller Types

#### **1. Forward and Inverse Differential Kinematics (Emphasized)**
These equations govern the motion of the robot based on its wheel velocities.

- **Wheel Angular Velocity:**
  $$
  \phi_{left} = \frac{\Delta p_{left}}{\Delta t}, \quad \phi_{right} = \frac{\Delta p_{right}}{\Delta t}
  $$

- **Linear and Angular Velocities:**
  $$
  v = \frac{r \phi_{right} + r \phi_{left}}{2}, \quad \omega = \frac{r \phi_{right} - r \phi_{left}}{d}
  $$

- **Position Update:**
  $$
  \Delta s = \frac{r \Delta p_{right} + r \Delta p_{left}}{2}
  $$
  $$
  \Delta \theta = \frac{r \Delta p_{right} - r \Delta p_{left}}{d}
  $$
  $$
  x = x + \Delta s \cos(\theta), \quad y = y + \Delta s \sin(\theta)
  $$

#### **2. Noisy Controller**
- Adds **Gaussian noise** to the wheel encoder readings:
  $$
  p'_{left} = p_{left} + \mathcal{N}(0, 0.005)
  $$
  $$
  p'_{right} = p_{right} + \mathcal{N}(0, 0.005)
  $$

### Launching the Controller
Run the following command to launch the controller:
```bash
ros2 launch bumperbot_controller controller.launch.py use_python:=true use_simple_controller:=true
```
- Set `use_python:=false` to use the C++ version.
- Set `use_simple_controller:=false` to use the noisy controller.

---

## Bumperbot Localization
The `bumperbot_localization` package fuses **IMU** and **wheel encoder** data using **Kalman and Extended Kalman Filters**.

### IMU Republisher
The `imu_republisher_node` repackages the IMU data:
- The IMU data frame is transformed to `base_footprint_ekf`.
- Publishes to the `imu_ekf` topic.

### Kalman Filter Implementation
- **Prediction Step:**
  $$
  \hat{x} = x + u, \quad \hat{P} = P + Q
  $$
- **Measurement Update:**
  $$
  K = \frac{P}{P + R}
  $$
  $$
  x = \hat{x} + K(z - \hat{x}), \quad P = (1 - K)P
  $$
- **Fusion of IMU and Wheel Encoder Data** to reduce noise and improve odometry.

### Launching Localization
Run the localization stack:
```bash
ros2 launch bumperbot_localization local_localization.launch.py
