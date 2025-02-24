# ROS Docker Monitor

## Overview
This repository provides a **ROS-based Docker monitoring system** that collects real-time performance metrics from running Docker containers and publishes them as ROS messages. The system consists of:

- A **ROS publisher (`docker_stats_publisher.py`)** that fetches CPU, memory, network, and process stats from Docker and publishes them to a ROS topic.
- A **custom ROS message (`DockerStats.msg`)** to structure the container statistics.

## Features
✅ Monitors **multiple running containers** simultaneously  
✅ Publishes **CPU, memory, network I/O, block I/O, and PIDs** statistics  
✅ Uses **multi-threading** for fast data collection  
✅ Adds a **timestamp** to each published message for synchronization  

---

## Installation & Setup
### **1. Prerequisites**
Ensure you have **ROS (Robot Operating System)** and **Docker** installed on your Ubuntu system.

```bash
sudo apt update && sudo apt install -y docker.io
```

### **2. Clone the Repository**
Ensure you are in your ROS workspace (e.g., `~/catkin_ws`).
```bash
cd ~/catkin_ws/src
git clone https://github.com/achilleas2942/docker_monitor.git
cd docker_monitor
```

### **3. Build the ROS Package**
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
or if you use catkin build
```bash
cd ~/catkin_ws
catkin build
source devel/setup.bash
```
---

## Usage
### **1. Start ROS Core**
```bash
roscore
```

### **2. Run the Docker Stats Publisher**
This script collects and publishes container statistics:
```bash
rosrun docker_monitor docker_stats_publisher.py
```

### **3. View Published Messages**
You can inspect the data being published with:
```bash
rostopic echo /docker_stats
```

---

## Custom Message Definition (`DockerStats.msg`)
This custom message defines the structure of the published Docker statistics:
```plaintext
string container_id
string name
float32 cpu_usage
float32 memory_usage
float32 memory_limit
float32 memory_percentage
string net_io
string block_io
int32 pids
time timestamp
```