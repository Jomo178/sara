# Semi-Autonomous Racing Automobile (SARA)
**Project Goal:** Develop a semi-autonomous racing automobile as part of a school project.

## Installation and Build Instructions

To install the repository and build the SARA project packages, follow these steps:

1. **Install ROS 2 Foxy:**
     Follow the installation instructions provided [here](https://docs.ros.org/en/foxy/Installation.html).

2. **Clone the repository:**
      ```bash
      git clone --recurse-submodules https://github.com/Jomo178/sara.git
      cd sara
      ```

## Building Packages

4. **Build the drive package:**
      ```bash
      colcon build
      ``` 

## Launch the SARA
      ```bash
      ros2 run drive drive_node 
      ros2 launch drive drive_launch.py
      ``` 