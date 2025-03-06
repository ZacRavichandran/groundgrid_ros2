import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return launch.LaunchDescription([
        # Declare the point_cloud_topic argument
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/ouster/points',
            description='Point cloud topic to subscribe to'
        ),

        # Component container with a unique name
        launch_ros.actions.ComposableNodeContainer(
            name='groundgrid_container_node', 
            namespace='groundgrid_ns',  
            package='rclcpp_components',
            executable='component_container',
            output='screen',
            composable_node_descriptions=[
                launch_ros.descriptions.ComposableNode(
                    package='groundgrid',
                    plugin='groundgrid::GroundGridNode',
                    name='groundgrid_node',  
                    remappings=[
                        ('/sensors/velodyne_points', LaunchConfiguration('point_cloud_topic'))
                    ]
                )
            ]
        ),
    ])
