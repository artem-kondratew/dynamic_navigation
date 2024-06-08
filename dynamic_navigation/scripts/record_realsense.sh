#! /bin/bash

ros2 bag record /camera/color/camera_info /camera/color/image_raw camera/aligned_depth_to_color/image_raw /tf_static /imu/data
