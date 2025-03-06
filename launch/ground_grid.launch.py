import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LoadComposableNodes

def generate_launch_description():
    container_name = 'groundgrid_component_container'
    namespace = 'groundgrid_ns'

    return launch.LaunchDescription([
        # Declare the point_cloud_topic argument
        DeclareLaunchArgument(
            'point_cloud_topic',
            default_value='/ouster/points',
            description='Point cloud topic to subscribe to'
        ),

        # Component container with a unique name
        launch_ros.actions.ComposableNodeContainer(
            name=container_name, 
            namespace=namespace, 
            package='rclcpp_components',
            executable='component_container',
            output='screen'
        ),

        # Load the component explicitly
        # LoadComposableNodes(
        #     target_container=f'/{namespace}/{container_name}',
        #     composable_node_descriptions=[
        #         launch_ros.descriptions.ComposableNode(
        #             package='groundgrid',
        #             plugin='groundgrid::GroundGridNode',
        #             name='groundgrid_node',
        #             remappings=[
        #                 ('/sensors/velodyne_points', LaunchConfiguration('point_cloud_topic'))
        #             ]
        #         )
        #     ]
        # )
    ])
