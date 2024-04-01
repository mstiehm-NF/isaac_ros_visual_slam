import launch
import yaml
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ComposableNode
def generate_launch_description():
    """Launch file which brings up visual slam node configured for OAK."""
    launch_args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the nodes'
        ),
        ExecuteProcess(
            cmd=['echo', 'Namespace is:', LaunchConfiguration('namespace')],
            output='screen',
        ),
    ]

    # Namespace
    namespace = LaunchConfiguration('namespace')

    depthai_prefix = get_package_share_directory('depthai_ros_driver')
    nvidia_slam_prefix = get_package_share_directory('isaac_ros_visual_slam')
    oak_params_file = os.path.join(nvidia_slam_prefix, 'params', 'oak-d.yaml')
    imu_filter_param_config = os.path.join(nvidia_slam_prefix, 'params', 'imu_filter.yaml')

    # https://github.com/ros2/rclcpp/issues/715#issuecomment-490425249
    # Composable Nodes use different yaml parsing than a standalone node.
    # This code will load the parameters from the yaml (removing the namespace/nodename/ros__parameters heading) so
    # that the parameters are parsed and named properly for the composable node.
    with open(imu_filter_param_config, 'r') as f:
        params = yaml.safe_load(f)['imu_filter']['ros__parameters']

    depthai = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(depthai_prefix, 'launch', 'camera.launch.py')),
            launch_arguments={"name": 'oak',
                              "params_file": oak_params_file,
                              "namespace": namespace
                               }.items())
    
    filter_node = ComposableNode(
                package='imu_filter_madgwick',
                plugin='ImuFilterMadgwickRos', 
                name='imu_filter_madgwick_node',
                namespace= namespace,
                parameters=[params],
                remappings=[
                    ('imu/data_raw', 'oak/imu/data'),
                    ('imu/data', 'imu/filtered_data')
                ]
            )

    oakd_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            filter_node
        ],
        output='screen'
    )

    final_launch_description = launch_args + [oakd_launch_container, depthai]
    return launch.LaunchDescription(final_launch_description)