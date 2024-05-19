from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    parameters=[{
          'Kp/MaxFeatures' : '10000',
          'frame_id':'base_link',
          'subscribe_depth':True,
          'subscribe_odom_info':False,
          'approx_sync':False,
          'wait_imu_to_init':True,}]

    remappings=[
          ('imu', '/imu/data'),
          ('rgb/image', '/rtabmap/yolo/rgb'),
          ('rgb/camera_info', '/rtabmap/yolo/camera_info'),
          ('depth/image', '/rtabmap/yolo/depth'),
          ('map', 'rtabmap/map'),
          ('odom', '/vo')]

    return LaunchDescription([

        # Nodes to launch       

        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            parameters=parameters,
            remappings=remappings),
    ])
