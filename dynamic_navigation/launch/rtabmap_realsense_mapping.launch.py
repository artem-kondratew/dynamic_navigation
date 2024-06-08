from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
        'Kp/MaxFeatures' : '10000',
        'frame_id':'camera_link',
        'subscribe_depth':True,
        'subscribe_odom_info':False,
        'approx_sync':False,
        'wait_imu_to_init':True,
        'odom_frame_id' : 'odom',
        # 'guess_frame_id' : 'odom',
        'Vis/FeatureType' : '8',
        }]

    remappings=[
        ('imu', '/imu/data'),
        ('rgb/image', '/rtabmap/yolo/rgb'),
        ('rgb/camera_info', '/rtabmap/yolo/camera_info'),
        ('depth/image', '/rtabmap/yolo/depth'),
        ('map', 'rtabmap/map'),
        ('odom', '/odom'),
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
