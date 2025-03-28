# Semi-Autonomous Racing Automobile (SARA)
**Project Goal:** Develop a semi-autonomous racing automobile as part of a school project.

## Installation and Build Instructions

To install the repository and build the SARA project packages, follow these steps:

1. **Install ROS 2 Foxy:**
     Follow the installation instructions provided [here](https://docs.ros.org/en/foxy/Installation.html).

2. **Clone the repository:**
      ```bash
      git clone https://github.com/Jomo178/sara.git
      cd sara
      ```

3. **Install ldrobot-lidar-ros2:**
      We use the DToF 2D Lidar sensor [LD19](https://www.waveshare.com/wiki/DTOF_LIDAR_LD19) for this project. Therefore, the scan listener package based on the [ldrobot-lidar-ros2](https://github.com/ldrobotSensorTeam/ldrobot-lidar-ros2) repository must be installed and built.

## Building Packages

4. **Build the drive package:**
      ```bash
      cd drive
      pip install .
      ``` 