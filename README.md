# ROS 2 Jazzy & NS-3 Integration on Ubuntu 24.04

## 📡 Bridging Real-time Robotics and Network Simulation

This repository demonstrates an integration between **ROS 2 Jazzy** and **NS-3 (v3.44)** on **Ubuntu 24.04**, enabling real-time communication between robotic applications and simulated network environments. It provides automation scripts for setting up both environments and showcases communication using **UDP over a Tap Bridge**, allowing ROS 2 nodes to interact with NS-3 simulated nodes as if they were part of the same network.

---

# 1.0 Initial Setup

## **Clone the Repository**
```bash
git clone <your-repo-url>
cd <your-repo-name>
```
# 1.1 ROS 2 Jazzy Development Setup

This repository contains a shell script to automate the setup of **ros2_jazzy.sh** development environment on **Ubuntu 24.04**.

##  Features
- Automatically sets up the ROS 2 Jazzy repository
- Installs required dependencies (`ros-dev-tools`, `python3-vcstool`, `colcon`, etc.)
- Creates a development workspace (`~/ros2_jazzy/src`)
- Imports the ROS 2 source code for development

---

##  Installation & Usage
- Run ros2_jazzy.sh on your bash terminal which is inside `setup` folder
```bash
chmod +x ros2_jazzy_setup.sh
./ros2_jazzy_setup.sh
```
- Source the ROS2 setup each time you open a new terminal
```bash
source /opt/ros/jazzy/setup.bash
```

## Notes
- This script is intended for Ubuntu 22.04 (Jammy).
- If you encounter issues with GPG keys, try running:
```bash
sudo rm -f /etc/apt/keyrings/ros-archive-keyring.gpg
```

# 1.2 NS-3.44 Development Setup

This repository contains a shell script to automate the setup of **ns-3.44.sh** development environment on **Ubuntu 24.04**.

## Features 

- Automatically downloads NS-3.44
- Installs required dependencies
- Extracts & builds NS-3.44

---

## Installation and Usage 
- Run ns-3.44.sh on your bash terminal which is inside `setup` folder
```bash
chmod +x ns3_setup.sh
./ns3_setup.sh
```




# 2.0 Project Structure & Source Files

The src/ folder contains subfolders for both ROS 2 nodes and NS-3 simulation scripts. Here's what you'll find:

```arduino
src/
├── ros2_code/
│   ├── turtle_circle_mover.py
│   └── turtle_pose_publisher.py
└── ns3_code/
    └── turtle_publisher.cc
```
## 🔁 Integration Technique
- The Tap Bridge module in NS-3 is used to connect a simulated NS-3 node with the host machine's network stack.

- This enables real-time UDP packet exchange between a ROS 2 node and a simulated NS-3 node.

- The turtle_pose_publisher.py node listens to /turtle1/pose, packs the pose data, and sends it over UDP to an NS-3 server using raw sockets.

## 🐢 ROS 2 Nodes
- turtle_circle_mover.py
    - Publishes velocity commands to move the turtle in a circle using /turtle1/cmd_vel.

- turtle_pose_publisher.py
    - Subscribes to /turtle1/pose and transmits the turtle's pose (x, y, θ) to NS-3 over UDP.

## 🌐 NS-3 Application
- udp_tapbridge_example.cc
    - Sets up a 4-node CSMA network.

    - Attaches a Tap Bridge to node 0, bridging it with the host's tap device (thetap).

    - Node 1 runs a UDP server that receives data from the ROS 2 host.

    - Uses RealtimeSimulatorImpl to synchronize simulation time with real-world time.



# 3.0 ✅ Future Enhancements
- Implement NS-3 to ROS 2 feedback communication

- Add Docker-based setup for platform-independent deployment

- Expand to multi-node robotic network scenarios


