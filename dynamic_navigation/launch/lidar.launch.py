import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


lidar_transform = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='lidar_static_transform_publisher',
    arguments=[
        '--x', '0.000',
        '--y', '0.000',
        '--z', '0.252',
        '--roll', '0.0',
        '--pitch', '0.0',
        '--yaw', '3.14',
        '--frame-id', 'base_footprint',
        '--child-frame-id', 'laser',
    ]
)


lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('rplidar_ros'), 'launch', 'rplidar_a1_launch.py')]
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )


def generate_launch_description():
    ld = LaunchDescription()

    ld.add_action(lidar_transform)
    ld.add_action(lidar)
    
    return ld
