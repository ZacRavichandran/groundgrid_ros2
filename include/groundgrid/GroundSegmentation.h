#pragma once

// ROS 2
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "velodyne_pointcloud/point_types.h"

// Grid Map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <groundgrid/GroundGridConfig.h>

namespace groundgrid {
class GroundSegmentation {
  public:
    using PCLPoint = velodyne_pointcloud::PointXYZIR;

    GroundSegmentation() = default;
    void init(const size_t dimension, const float& resolution);
    pcl::PointCloud<PCLPoint>::Ptr filter_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const PCLPoint& cloudOrigin, const geometry_msgs::msg::TransformStamped& mapToBase, grid_map::GridMap &map);
    void insert_cloud(const pcl::PointCloud<PCLPoint>::Ptr cloud, const size_t start, const size_t end, const PCLPoint& cloudOrigin, std::vector<std::pair<size_t, grid_map::Index> >& point_index, std::vector<std::pair<size_t, grid_map::Index> >& ignored, std::vector<size_t>& outliers, grid_map::GridMap &map);
    void detect_ground_patches(grid_map::GridMap &map, unsigned short section) const;
    template<int S> void detect_ground_patch(grid_map::GridMap &map, size_t i, size_t j) const;
    void spiral_ground_interpolation(grid_map::GridMap &map, const geometry_msgs::msg::TransformStamped &toBase) const;
    void interpolate_cell(grid_map::GridMap &map, const size_t x, const size_t y) const;

  protected:
    GroundGridConfig mConfig;
    grid_map::Matrix expectedPoints;

    const float verticalPointAngDist = 0.00174532925*2;
    const float minDistSquared = 12.0f;
};
}
