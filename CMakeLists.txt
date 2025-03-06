cmake_minimum_required(VERSION 3.5)
project(groundgrid)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_cv REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_visualization REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3 REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
    src/GroundGrid.cpp
)
ament_target_dependencies(${PROJECT_NAME}_lib
    rclcpp
    tf2
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    grid_map_core
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    grid_map_rviz_plugin
)

add_library(${PROJECT_NAME}_groundsegmentation_lib
    src/GroundSegmentation.cpp
)
ament_target_dependencies(${PROJECT_NAME}_groundsegmentation_lib
    rclcpp
    tf2
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    pcl_ros
    pcl_conversions
    grid_map_ros
    grid_map_cv
    grid_map_msgs
)

add_library(groundgrid_component SHARED src/GroundGridNode.cpp)
ament_target_dependencies(groundgrid_component
    rclcpp
    rclcpp_components
    tf2
    tf2_ros
    tf2_msgs
    tf2_geometry_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    std_msgs
    cv_bridge
    image_transport
    pcl_ros
    pcl_conversions
    grid_map_core
    grid_map_ros
    grid_map_cv
    grid_map_msgs
    grid_map_visualization
)

target_link_libraries(groundgrid_component
    ${PROJECT_NAME}_lib
    ${PROJECT_NAME}_groundsegmentation_lib
)

rclcpp_components_register_node(
    groundgrid_component
    PLUGIN "groundgrid::GroundGridNode"
    EXECUTABLE groundgrid_node
)
ament_export_targets(export_groundgrid_component)
install(TARGETS 
        ${PROJECT_NAME}_lib
        ${PROJECT_NAME}_groundsegmentation_lib
        ${PROJECT_NAME}_node
        groundgrid_component
        EXPORT export_groundgrid_component
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include/
    FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

ament_package()
