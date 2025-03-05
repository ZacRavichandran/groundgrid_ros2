import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/sensors/velodyne_points',
            description='Point cloud topic to subscribe to'
        ),
        DeclareLaunchArgument(
            'use_nodelets',
            default_value='false',
            description='Use nodelets (not applicable in ROS 2)'
        ),
        DeclareLaunchArgument(
            'nodelet_manager',
            default_value='core_nodelet_manager',
            description='Nodelet manager (not applicable in ROS 2)'
        ),

        # Remap topic
        launch_ros.actions.Node(
            package='groundgrid',
            executable='groundgrid_node',
            name='groundgrid',
            remappings=[
                ('/sensors/velodyne_points', LaunchConfiguration('point_cloud_topic'))
            ],
            condition=launch.conditions.UnlessCondition(LaunchConfiguration('use_nodelets'))
        ),
    ])
