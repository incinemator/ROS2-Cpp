# RoboNavigation
Autonomous navigation algorithms for mobile robotic platforms equipped with RGBD or depth cameras and GPS written in C++ for ROS2. 
## Packages
### `jackal_vslam`
### `orb_vslam_ros`
### `turtlebot_lidar_slam1`
#### ROS2 Version
Humble
#### Dependencies

- `rclcpp`: ROS2 C++ client library.
- `sensor_msgs`: Standard ROS2 messages for sensor data.
- `geometry_msgs`: Standard ROS2 messages for geometric data.
- `nav_msgs`: Standard ROS2 messages for navigation data.

## Installation

1. Clone this repository into your ROS2 workspace:

   ```bash
   git clone <repository_url> src/ros2_navigation
## Usage
source /opt/ros/humble/setup.bash
ros2 run turtlebot3_navigation turtlebot3_navigation
