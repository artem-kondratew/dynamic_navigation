import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'dynamic_navigation'

    params_file = 'dynamic_controller_params_gazebo.yaml'
    parameters = [os.path.join(get_package_share_directory(package_name), 'config', params_file)]

    dynamic_controller = Node(
        package=package_name,
        executable='dynamic_controller',
        parameters=parameters,
        output='screen',
    )

    return LaunchDescription([
        dynamic_controller,
    ])
