# Vehicle Controller System (ROS 2)

## 🚗 Overview

This ROS 2-based project simulates a vehicle controller that:

- Receives real-time GPS and velocity data
- Predicts and generates safe trajectories based on traffic light state and distance to intersection
- Publishes control commands as `TwistStamped` messages
- Logs predicted and actual vehicle trajectories for analysis

---

## 📦 Packages

### 1. `vehicle_controller`
- Core logic for trajectory generation and vehicle control
- Includes signal input handling, socket communication, and control loop
- Parameters for traffic light cycle and vehicle dynamics are configurable

### 2. `vehicle_sender`
- communicate with vehicle control hardware

---

## 🛠 Features

- ✅ Sensor fusion using GPS and velocity (BESTVEL)
- ✅ Portabel trajectory generation module
- ✅ Traffic light logic with red/green/yellow phase awareness
- ✅ Acceleration phase to 5 m/s before normal control starts
- ✅ CSV logging for both predicted and actual trajectories
- ✅ Socket interface for SPaT (Signal Phase and Timing) input

---

## 🧪 Dependencies

- ROS 2 (Humble or later)
- `geometry_msgs`, `rclcpp`
- `novatel_oem7_msgs` (for GPS/velocity input)

---

## 🚀 How to Build

```bash
cd ~/catkin_ws_resilience
colcon build
source install/setup.bash
