# Localization Technology Development and Implementation

## Overview

This repository contains the code and documentation for a self-localization and navigation system developed for a ground robot. The system leverages onboard sensors to enable autonomous navigation, utilizing advanced technologies and tools for robust performance.

## Project Description

- **System Development**: Created a system for self-localization and autonomous navigation using onboard sensors.
- **Technology Integration**: Employed Micro ROS and the Rosserial library to interface sensors with ESP32 and Arduino for low-level control.
- **Tools and Technologies**: 
  - ROS2 (Robot Operating System 2)
  - OpenCV
  - SLAM (Simultaneous Localization and Mapping)
  - Navigation Algorithms

## Features

- **Autonomous Navigation**: Enables the robot to navigate independently using sensor data for localization.
- **Sensor Integration**: Interfaces with sensors through ESP32 and Arduino using Micro ROS and Rosserial.
- **Advanced Algorithms**: Utilizes ROS2, OpenCV, and SLAM for efficient mapping and navigation.

## Repository Contents

- **`src/`**: Source code for the localization and navigation system.
- **`config/`**: Configuration files for system setup and parameter tuning.
- **`docs/`**: Documentation including setup instructions, usage guidelines, and system architecture.
- **`examples/`**: Sample code and example configurations demonstrating system functionality.

## Installation
1. **Make a src folder**:
Make a source folder in you workspace
   ```bash
   mkdir src
   cd src
   ```
2. **Clone the Repository**:
   ```bash
   git clone https://github.com/Vedhamshbode/bbot_description.git
   ```
3. **Install Dependencies**:
Ensure you have ROS2, OpenCV, and other required libraries installed on your system. Follow the installation instructions provided in the docs/ directory.
4. **Build the Project**:
Navigate to the root of the repository and build the project:
   ```bash
   cd bbot_description
   colcon build
   ```
5. **Setup and Configuration**:
Modify the configuration files in the config/ directory as needed to match your hardware setup. For example, you may need to adjust the sensor calibration parameters.

## Usage
1. **Launch the system**:
   ```bash
   ros2 launch bbot_description launch_sim.launch.py
   ```
2. **Monitor and Control**:
Use ROS2 tools and interfaces to monitor the robot’s status and control its navigation. For example, you can visualize the robot’s path using:
```bash
ros2 run rviz2 rviz2
```
