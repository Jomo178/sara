# Semi-Autonomous Racing Automobile (SARA)
**Project Goal:** Develop a semi-autonomous racing automobile as part of a school project.

## Installation and Build Instructions

To install the repository and build the SARA project packages, follow these steps:

1. **Install ROS 2 Jazzy:**
    Follow the installation instructions provided [here](https://docs.ros.org/en/jazzy/Installation.html).

2. **Clone the repository:**
     ```bash
     git clone https://github.com/Jomo178/sara.git
     cd sara
     ```
     
## Building Packages

3. **Build the drive package:**
     ```bash
     cd drive
     pip install .
     ```

4. **Build the scan listener package:**
    We use the DToF 2D Lidar sensor [LD19](https://www.waveshare.com/wiki/DTOF_LIDAR_LD19) for this project, and therefore the scan listener repository is required. It is already included in the project. The scan listener package is based on the [ldrobot-lidar-ros2](https://github.com/Myzhar/ldrobot-lidar-ros2) repository. You just need to build it:
     ```bash
     cd ../ldrobot-lidar-ros2
     colcon build
     ```
     
## License

This project includes software from the [Myzhar/ldrobot-lidar-ros2](https://github.com/Myzhar/ldrobot-lidar-ros2) repository, which is licensed under the LICENSE file provided in that repository. Please refer to the [LICENSE](https://github.com/Myzhar/ldrobot-lidar-ros2/blob/devel/LICENSE) for more details.
