#ifndef DYNAMIC_DETECTOR_DEPTH_CONVERSIONS_HPP
#define DYNAMIC_DETECTOR_DEPTH_CONVERSIONS_HPP


#include <limits>

#include <opencv4/opencv2/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/msg/point.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>


namespace depthimage_to_pointcloud2 {

std::vector<geometry_msgs::msg::Point> convert(const sensor_msgs::msg::Image& depth_msg, sensor_msgs::msg::PointCloud2::SharedPtr & cloud_msg,
                                                                                                    const image_geometry::PinholeCameraModel & model, double range_max) {

    // Use correct principal point from calibration
    float center_x = model.cx();
    float center_y = model.cy();

    // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
    float constant_x = 1 / model.fx();
    float constant_y = 1 / model.fy();

    sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg.data[0]);
    int row_step = depth_msg.step / sizeof(float);
    std::vector<geometry_msgs::msg::Point> xyz;

    for (int v = 0; v < static_cast<int>(cloud_msg->height); ++v, depth_row += row_step) {
        for (int u = 0; u < static_cast<int>(cloud_msg->width); ++u, ++iter_x, ++iter_y, ++iter_z) {
            float depth = depth_row[u];

            if (!std::isfinite(depth) || depth > range_max) {
                depth = range_max;
            }

            *iter_x = (u - center_x) * depth * constant_x;
            *iter_y = (v - center_y) * depth * constant_y;
            *iter_z = depth;

            geometry_msgs::msg::Point pt;
            pt.x = *iter_x;
            pt.y = *iter_y;
            pt.z = *iter_z;
            xyz.push_back(pt);
        }
    }
    return xyz;
}

}


#endif  // DYNAMIC_DETECTOR_DEPTH_CONVERSIONS_HPP
