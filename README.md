# ROS2 Jazzy Implementation
This is a migrated version of GroundGrid for ROS2 Jazzy.

# Source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation"
This repository contains the source code for the article "GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation" published in the IEEE Robotics and Automation Letters ([DOI: 10.1109/LRA.2023.3333233](https://doi.org/10.1109/lra.2023.3333233)).
<p align="center">
  <img src="/res/img/teaser.gif" alt="Ground segmentation results"/>
</p>

# Dependencies
- ROS2 Jazzy
- colcon
- rclcpp
- rclcpp_components
- geometry_msgs
- sensor_msgs
- std_msgs
- tf2
- tf2_ros
- tf2_msgs
- tf2_geometry_msgs

- grid_map_core
- grid_map_ros
- grid_map_cv
- grid_map_msgs
- grid_map_visualization
- grid_map_rviz_plugin
- cv_bridge
- nav_msgs
- pcl_ros


# Build
```
cd ~/ros2_ws/src
git clone https://github.com/frankgon1627/groundgrid.git
cd ..
colcon build -DCMAKE_BUILD_TYPE=Release --packages-select groundgrid
```

# Launch
## Playback
```
source ~/ros2_ws/install/setup.bash
ros2 launch groundgrid ground_grid.launch.py
```

# Limitations
Currently unable to run with grid_map_rviz_plugin so cannot visualize the image outputs. Semantic pointcloud however,
is visible. Not tested against original benchmarks

# Citation
```
@article{steinke2024groundgrid,
  author={Steinke, Nicolai and Goehring, Daniel and Rojas, Raúl},
  journal={IEEE Robotics and Automation Letters},
  title={GroundGrid: LiDAR Point Cloud Ground Segmentation and Terrain Estimation},
  year={2024},
  volume={9},
  number={1},
  pages={420-426},
  keywords={Sensors;Point cloud compression;Estimation;Laser radar;Image segmentation;Task analysis;Robot sensing systems;Range Sensing;Mapping;Field Robots},
  doi={10.1109/LRA.2023.3333233}}
```
