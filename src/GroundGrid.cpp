#include <groundgrid/GroundGrid.h>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <grid_map_core/GridMapMath.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace groundgrid;
    
GroundGrid::GroundGrid(tf2_ros::Buffer& tf_buffer, tf2_ros::TransformListener& tf_listener)
        : tf_buffer_(tf_buffer), tf_listener_(tf_listener) {}

GroundGrid::~GroundGrid() {}

void GroundGrid::initGroundGrid(const nav_msgs::msg::Odometry::SharedPtr inOdom) {
    std::chrono::_V2::steady_clock::time_point start = std::chrono::steady_clock::now();

    geometry_msgs::msg::PoseWithCovarianceStamped odomPose;

    mMap_ptr = std::make_shared<grid_map::GridMap>(std::vector<std::string>{"points", "ground", "groundpatch", "minGroundHeight", "maxGroundHeight"});
    grid_map::GridMap &map = *mMap_ptr;
    map.setFrameId("map");
    map.setGeometry(grid_map::Length(mDimension, mDimension), mResolution,
                    grid_map::Position(inOdom->pose.pose.position.x, inOdom->pose.pose.position.y));

    RCLCPP_INFO(rclcpp::get_logger("groundgrid"), "Created map with size %f x %f m (%i x %i cells).",
                map.getLength().x(), map.getLength().y(),
                map.getSize()(0), map.getSize()(1));

    odomPose.pose = inOdom->pose;
    odomPose.header = inOdom->header;
    std::vector<grid_map::BufferRegion> damage;
    map.move(grid_map::Position(odomPose.pose.pose.position.x, odomPose.pose.pose.position.y), damage);

    map["points"].setZero();
    map["ground"].setConstant(inOdom->pose.pose.position.z);
    map["groundpatch"].setConstant(0.0000001);
    map["minGroundHeight"].setConstant(100.0);
    map["maxGroundHeight"].setConstant(-100.0);

    std::chrono::_V2::steady_clock::time_point end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(rclcpp::get_logger("groundgrid"), "Transforms lookup took %ld ms",
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    mLastPose = odomPose;
}

std::shared_ptr<grid_map::GridMap> GroundGrid::update(const nav_msgs::msg::Odometry::SharedPtr inOdom) {
    if (!mMap_ptr) {
        initGroundGrid(inOdom);
        return mMap_ptr;
    }

    std::chrono::_V2::steady_clock::time_point start = std::chrono::steady_clock::now();
    grid_map::GridMap &map = *mMap_ptr;

    geometry_msgs::msg::PoseWithCovarianceStamped poseDiff;
    poseDiff.pose.pose.position.x = inOdom->pose.pose.position.x - mLastPose.pose.pose.position.x;
    poseDiff.pose.pose.position.y = inOdom->pose.pose.position.y - mLastPose.pose.pose.position.y;

    std::vector<grid_map::BufferRegion> damage;
    map.move(grid_map::Position(inOdom->pose.pose.position.x, inOdom->pose.pose.position.y), damage);

    // Static so if the new transform is not yet available, we can use the last one
    static geometry_msgs::msg::TransformStamped base_to_map;

    try {
        base_to_map = tf_buffer_.lookupTransform("base_link", "map", tf2::TimePointZero, std::chrono::milliseconds(1000));
    } catch (const tf2::LookupException &e) {
        RCLCPP_WARN(rclcpp::get_logger("groundgrid"), "No transform available: %s", e.what());
    } catch (const tf2::ExtrapolationException &e) {
        RCLCPP_DEBUG(rclcpp::get_logger("groundgrid"), "Extrapolation required: %s", e.what());
    }

    geometry_msgs::msg::PointStamped ps;
    ps.header = inOdom->header;
    ps.header.frame_id = "map";
    grid_map::Position pos;

    for (auto region : damage) {
        for (auto it = grid_map::SubmapIterator(map, region); !it.isPastEnd(); ++it) {
            auto idx = *it;

            map.getPosition(idx, pos);
            ps.point.x = pos(0);
            ps.point.y = pos(1);
            ps.point.z = 0;
            tf2::doTransform(ps, ps, base_to_map);
            map.at("ground", idx) = -ps.point.z;
            map.at("groundpatch", idx) = 0.0;
        }
    }

    // If no movement, no update needed
    if (damage.empty())
        return mMap_ptr;

    mLastPose.pose = inOdom->pose;
    mLastPose.header = inOdom->header;

    map.convertToDefaultStartIndex();
    std::chrono::_V2::steady_clock::time_point end = std::chrono::steady_clock::now();
    RCLCPP_DEBUG(rclcpp::get_logger("groundgrid"), "Total processing time: %ld ms",
                std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    return mMap_ptr;
}