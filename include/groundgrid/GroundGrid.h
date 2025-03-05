#pragma once

// Grid map
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>

// ROS 2 messages
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace groundgrid {

class GroundGrid {
public:
    GroundGrid();

    virtual ~GroundGrid();
    void initGroundGrid(const nav_msgs::msg::Odometry::SharedPtr inOdom);
    
    std::shared_ptr<grid_map::GridMap> update(const nav_msgs::msg::Odometry::SharedPtr inOdom);

    const float mResolution = .33f;
    const float mDimension = 120.0f;

private:
    tf2_ros::Buffer mTfBuffer;
    tf2_ros::TransformListener mTf2_listener;

    double mDetectionRadius = 60.0;
    std::shared_ptr<grid_map::GridMap> mMap_ptr;
    geometry_msgs::msg::TransformStamped mTfPosition, mTfLux, mTfUtm, mTfMap;
    geometry_msgs::msg::PoseWithCovarianceStamped mLastPose;
};

}
