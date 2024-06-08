import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('dynamic_navigation'), 'launch', 'realsense.launch.py')]
        )
    )

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('yolo'), 'launch', 'motion_detector.launch.py')]
        )
    )

    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('dynamic_detector'), 'launch', 'detect_realsense.launch.py')]
        )
    )

    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('dynamic_navigation'), 'launch', 'dynamic_controller_realsense.launch.py')]
        )
    )

    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('dynamic_navigation'), 'launch', 'rtabmap_realsense_localization.launch.py')]
        )
    )

    return LaunchDescription([
        realsense,
        yolo,
        detector,
        controller,
        rtabmap,
    ])
