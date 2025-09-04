from launch import LaunchDescription
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import LoadComposableNodes, Node

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    container_name = LaunchConfiguration('container_name', default='/ouster/os_container')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='warty/lidar_points')
    odom_topic = LaunchConfiguration('odom_topic', default='/warty/platform/odom')
    map_frame = LaunchConfiguration('map_frame', default='warty/odom')
    base_frame = LaunchConfiguration('base_frame', default='warty/base_link')
    lidar_frame = LaunchConfiguration('lidar_frame', default='warty/lidar_link')

    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
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
    declare_odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value=odom_topic,
        description='Odom topic name'
    )
    declare_map_frame_arg = DeclareLaunchArgument(
        'map_frame',
        default_value=map_frame,
        description='Map frame name'
    )
    declare_base_frame_arg = DeclareLaunchArgument(
        'base_frame',
        default_value=base_frame,
        description='Base frame name'
    )
    declare_lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame',
        default_value=lidar_frame,
        description='Lidar frame name'
    )

    # load_composable_nodes = LoadComposableNodes(
    #     target_container=[namespace, container_name],
    #     composable_node_descriptions=[
    #         ComposableNode(
    #             package='groundgrid',
    #             plugin='groundgrid::GroundGridNode',
    #             name='groundgrid_node',
    #             namespace=namespace,
    #             remappings=[
    #                 ('/sensors/velodyne_points', pointcloud_topic)
    #             ]
    #         )
    #     ]
    # )

    groundgrid_node = Node(
    package='groundgrid',
    executable='groundgrid_node',
    name='groundgrid_node',
    parameters=[{
        'pointcloud_topic': pointcloud_topic,
        'odom_topic': odom_topic,
        'map_frame': map_frame,
        'base_frame': base_frame,
        'lidar_frame': lidar_frame,
    }]
)

    return LaunchDescription([
        declare_namespace_arg,
        declare_pointcloud_topic_arg,
        declare_odom_topic_arg,
        declare_map_frame_arg,
        declare_base_frame_arg,
        declare_lidar_frame_arg,
        # declare_container_name_arg,
        groundgrid_node,
    ])
