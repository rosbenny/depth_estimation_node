# Depth Estimation Node (ROS 2)

> 🚗 Developed as part of an Emergency Braking System (EBS) project using ROS 2 by **Rose Benny**

---

## 📌 Project Description

This ROS 2 node estimates the **distance of an object** using either:
- **X and Y coordinates** (simulated or sensor input)
- **Depth image center pixel** from a camera (e.g., RealSense, Gazebo)

It publishes the calculated depth and logs safety warnings based on proximity.

---

## 🛠 Tools & Technologies Used

- **Ubuntu 22.04 (WSL)** – Linux environment on Windows
- **ROS 2 Humble Hawksbill**
- **Python 3**
- **rclpy** – ROS 2 Python client
- **colcon** – ROS 2 build system
- **std_msgs** and **sensor_msgs**
- **cv_bridge** (optional, for depth camera support)

---

## 📦 Package Structure

```
depth_estimation_node/
├── depth_estimation_node/
│   ├── __init__.py
│   └── depth_node.py
├── package.xml
├── setup.py
├── setup.cfg
└── test/
```

---

## 🧠 How It Works

### ➤ From XY Input:
Subscribes to `/xy_input` topic (Float32MultiArray) and calculates:

```
distance = sqrt(x² + y²)
```

### ➤ From Camera:
Subscribes to `/camera/depth/image_raw`, extracts center pixel depth using OpenCV, and publishes the result.

---

## 🚀 How to Build & Run

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 run depth_estimation_node depth_node
```

In another terminal, test it with:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub /xy_input std_msgs/Float32MultiArray "data: [3.0, 4.0]"
```

---

## 📈 Sample Output

```bash
[INFO] [depth_estimator]: ✅ [XY] Safe distance: 5.00 m
[WARN] [depth_estimator]: ⚠️ [Camera-Center] Obstacle too close! Distance: 0.45 m
```

---

## 📄 License

MIT License © 2025 Rose Benny

---

## 🙌 Acknowledgements

Created as part of the Prowler Team’s Autonomous Emergency Braking System mini project for academic research and robotics practice.

---
