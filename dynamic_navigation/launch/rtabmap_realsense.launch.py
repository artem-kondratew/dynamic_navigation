# Requirements:
#   A realsense D435i
#   Install realsense2 ros2 package (ros-$ROS_DISTRO-realsense2-camera)
# Example:
#   $ ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_sync:=true align_depth.enable
#
#   $ ros2 launch rtabmap_examples realsense_d435i_color.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
        'Kp/MaxFeatures' : '10000',
        'frame_id':'base_footprint',
        'subscribe_depth':True,
        'subscribe_odom_info':False,
        'approx_sync':False,
        'wait_imu_to_init':True,
        'odom_frame_id' : 'vo',
        'guess_frame_id' : 'odom',
        'Vis/FeatureType' : '8',
        }]

    remappings=[
        ('imu', '/imu/data'),
        ('rgb/image', '/rtabmap/yolo/rgb'),
        ('rgb/camera_info', '/rtabmap/yolo/camera_info'),
        ('depth/image', '/rtabmap/yolo/depth'),
        ('map', 'rtabmap/map'),
        ('odom', '/vo'),
        ]

    return LaunchDescription([

        # Nodes to launch       
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            parameters=parameters,
            remappings=remappings),

        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']),
    ])
