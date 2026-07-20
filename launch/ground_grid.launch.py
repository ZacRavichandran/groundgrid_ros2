from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    node = Node(
        package="groundgrid",
        executable="groundgrid_node",
        name="groundgrid_node",
        namespace=namespace,
        output="screen",
        parameters=[
            {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "z_threshold": LaunchConfiguration("z_threshold"),
                "transform_timeout": LaunchConfiguration("transform_timeout"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "pointcloud_topic": LaunchConfiguration("pointcloud_topic"),
                "grid_map_topic": LaunchConfiguration("grid_map_topic"),
                "segmented_cloud_topic": LaunchConfiguration(
                    "segmented_cloud_topic"
                ),
                "obstacle_cloud_topic": LaunchConfiguration(
                    "obstacle_cloud_topic"
                ),
                "odom_frame": LaunchConfiguration("odom_frame"),
                "base_frame": LaunchConfiguration("base_frame"),
                "lidar_frame": LaunchConfiguration("lidar_frame"),
                "utm_frame": LaunchConfiguration("utm_frame"),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("z_threshold", default_value="1000.0"),
            DeclareLaunchArgument("transform_timeout", default_value="0.5"),
            DeclareLaunchArgument(
                "odom_topic", default_value="dlio/odom_node/odom"
            ),
            DeclareLaunchArgument(
                "pointcloud_topic", default_value="ouster/points"
            ),
            DeclareLaunchArgument(
                "grid_map_topic", default_value="groundgrid/grid_map"
            ),
            DeclareLaunchArgument(
                "segmented_cloud_topic",
                default_value="groundgrid/segmented_cloud",
            ),
            DeclareLaunchArgument(
                "obstacle_cloud_topic",
                default_value="groundgrid/obstacle_cloud",
            ),
            DeclareLaunchArgument("odom_frame", default_value="odom"),
            DeclareLaunchArgument("base_frame", default_value="base_link"),
            DeclareLaunchArgument("lidar_frame", default_value="os_lidar"),
            DeclareLaunchArgument("utm_frame", default_value="utm"),
            DeclareLaunchArgument(
                "container_name",
                default_value="",
                description=(
                    "Deprecated compatibility argument; GroundGrid now runs "
                    "in its own process"
                ),
            ),
            node,
        ]
    )
