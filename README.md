# ROS2 QoS-Based Runtime Health Monitoring and Safety Fallback System

## Overview
This project implements a ROS2-based architecture for **runtime health monitoring and safe fallback control** using QoS concepts like **deadline, liveliness, and reliability**.

The system ensures the robot **does not operate under unsafe or unhealthy conditions** such as missing data, node failure, or delayed communication.

---

## Problems Addressed

Robotic systems face critical runtime issues:

- Nodes may stop publishing during execution  
- Messages may be delayed or lost  
- System may continue using stale data  
- No clear runtime failure detection  
- No automatic safe fallback during faults  

---

## Approaches Used

### 1. Separate Health Monitoring
A dedicated health monitor node observes all nodes instead of embedding checks inside each node.

### 2. QoS-Based Detection
Uses ROS2 QoS:
- **Deadline** → detect missing or delayed messages (partial implementation)  
- **Liveliness** → detect node failure  
- **Reliability** → ensure message delivery  

### 3. Centralized Safety Decision
All safety decisions use a **single threshold** in one node to avoid inconsistencies.

### 4. Safe Fallback Execution
System reacts automatically:
- Unsafe → reduce speed  
- Failure → stop system  

### 5. Redundancy
Backup health monitor prevents single point of failure.

---

## Architecture
The system consists of:

- Sensor Nodes  
- Safety Decision Node  
- Health Monitor (Primary + Backup)  
- Safety Action Node  
- Status Display  

---

## Nodes

### Sensor A – Distance
Publishes:
- `/data_a`
- `/node1/heartbeat`

---

### Sensor B – Speed
Publishes:
- `/data_b`
- `/node2/heartbeat`

---

### Safety Decision Node
Subscribes:
- `/data_a`, `/data_b`

Publishes:
- `/system_status`
- `/adjusted_speed`
- `/node3/heartbeat`

Logic:
- distance ≥ threshold → SAFE  
- distance < threshold → REDUCE SPEED  
- missing data → STOP  

---

### Health Monitor Node
Monitors:
- heartbeats  
- data topics  

Publishes:
- `/health_status`

Detects:
- node failure  
- data timeout  
- system unhealthy  

---

### Health Monitor Backup
- Redundant monitoring node  
- Ensures robustness  

---

### Safety Action Node
Subscribes:
- `/system_status`, `/health_status`

Publishes:
- `/final_robot_action`

Outputs:
- NORMAL / SLOW / STOP  

---

### Status Display Node
Displays:
- SYSTEM  
- HEALTH  
- ACTION  

---

### Web Server Node
Serves a browser dashboard for the runtime health and safety system.

Subscribes:
- `/data_a`, `/data_b`
- `/camera/object_status`
- `/system_status`, `/adjusted_speed`
- `/health_status`, `/health_reason`
- `/health_status_backup`, `/health_reason_backup`
- `/node1/heartbeat`, `/node2/heartbeat`, `/node3/heartbeat`, `/node6/heartbeat`

Serves:
- `http://localhost:8080/` live dashboard
- `http://localhost:8080/api/status` JSON status snapshot
- `http://localhost:8080/events` live Server-Sent Events stream

---

### Camera Node (Optional)
- Additional perception input  

---

### Node7 Test Node
- Used for testing system behavior  

---

## Topics

| Topic | Purpose |
|------|--------|
| /data_a | Distance |
| /data_b | Speed |
| /node*/heartbeat | Node health |
| /system_status | Safety decision |
| /health_status | Health state |
| /final_robot_action | Final action |

---

## QoS Usage

- **Reliability** → ensures message delivery  
- **Deadline** → detects missing or delayed messages (conceptual / partial)  
- **Liveliness** → detects node failure  

---

## Safety Logic

- Safe → Normal operation  
- Unsafe distance → Reduce speed  
- Failure / missing data → Stop system  

---

## Build

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

## Run 

```bash
source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash && \
ros2 run qos_health_monitor_demo sensor_a_node & \
ros2 run qos_health_monitor_demo sensor_b_node & \
ros2 run qos_health_monitor_demo safety_decision_node & \
ros2 run qos_health_monitor_demo health_monitor_node --ros-args -p config_file:=/home/subash/ros2_ws/src/qos-health-monitor-ros2/config/health_monitor.yaml & \
ros2 run qos_health_monitor_demo health_monitor_backup_node --ros-args -p config_file:=/home/subash/ros2_ws/src/qos-health-monitor-ros2/config/health_monitor.yaml & \
ros2 run qos_health_monitor_demo status_display_node & \
ros2 run qos_health_monitor_demo web_server_node & \
ros2 run qos_health_monitor_demo camera_node & \
ros2 run qos_health_monitor_demo node7_test_node
```

Open the dashboard:

```text
http://localhost:8080/
```

To use another port:

```bash
ros2 run qos_health_monitor_demo web_server_node --ros-args -p port:=8090
```

---

## Tech Stack

- ROS2 Jazzy  
- Ubuntu 24.04  
- C++ (rclcpp)  
