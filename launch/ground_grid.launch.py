from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LoadComposableNodes

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    container_name = LaunchConfiguration('container_name', default='/ouster/os_container')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='ouster/points')

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='j100_0000',
        description='Namespace for all nodes and topics'
    )

    declare_container_name_arg = DeclareLaunchArgument(
        'container_name',
        default_value=container_name,
        description='Name of the container to load nodes into'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=[namespace, container_name],
        composable_node_descriptions=[
            ComposableNode(
                package='groundgrid',
                plugin='groundgrid::GroundGridNode',
                name='groundgrid_node',
                namespace=namespace,
                remappings=[
                    ('/sensors/velodyne_points', pointcloud_topic)
                ]
            )
        ]
    )

    return LaunchDescription([
        declare_namespace_arg,
        declare_pointcloud_topic_arg,
        declare_container_name_arg,
        load_composable_nodes,
    ])
