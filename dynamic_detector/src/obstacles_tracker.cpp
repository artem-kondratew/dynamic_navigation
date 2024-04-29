#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

#include "dynamic_nav_msgs/msg/obstacles_footprints.hpp"


class ObstaclesTracker : public rclcpp::Node {
private:
    const int IMG_UNOCCUPIED = 255;
    const int IMG_OCCUPIED = 0;
    const int IMG_UNKNOWN = 128;
    int MAP_UNOCCUPIED;
    int MAP_OCCUPIED;
    int MAP_UNKNOWN;
    
    rclcpp::Subscription<dynamic_nav_msgs::msg::ObstaclesFootprints>::SharedPtr fp_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

    std::string map_frame_, camera_link_optical_frame_;
    bool track_;
    float obstacle_offset_;
    cv::Mat img_map_;
    nav_msgs::msg::OccupancyGrid map_;
    tf2::Transform tf_;

public:
    ObstaclesTracker();

private:
    visualization_msgs::msg::Marker create2dBoxMsg(std::vector<geometry_msgs::msg::Point> points);

    void footprintsCallback(const dynamic_nav_msgs::msg::ObstaclesFootprints msg);
    void mapCallback(const nav_msgs::msg::OccupancyGrid msg);
    void transformCallback();
};


ObstaclesTracker::ObstaclesTracker() : Node("obstacles_tracker") {
    using namespace std::chrono_literals;
    using std::placeholders::_1;

    this->declare_parameter("footprints_topic", "");
    this->declare_parameter("map_sub_topic", "");
    this->declare_parameter("map_pub_topic", "");
    this->declare_parameter("map_frame", "");
    this->declare_parameter("camera_link_optical_frame", "");
    this->declare_parameter("track_obstacles", true);
    this->declare_parameter("obstacle_offset", 0.0);
    this->declare_parameter("unoccupied_value", 0);
    this->declare_parameter("occupied_value", 0);
    this->declare_parameter("unknown_value", 0);

    std::string fp_topic = this->get_parameter("footprints_topic").as_string();
    std::string map_sub_topic = this->get_parameter("map_sub_topic").as_string();
    std::string map_pub_topic = this->get_parameter("map_pub_topic").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    camera_link_optical_frame_ = this->get_parameter("camera_link_optical_frame").as_string();
    track_ = this->get_parameter("track_obstacles").as_bool();
    obstacle_offset_ = this->get_parameter("obstacle_offset").as_double();
    MAP_UNOCCUPIED = this->get_parameter("unoccupied_value").as_int();
    MAP_OCCUPIED = this->get_parameter("occupied_value").as_int();
    MAP_UNKNOWN = this->get_parameter("unknown_value").as_int();

    RCLCPP_INFO(this->get_logger(), "footprints_topic: '%s'", fp_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "map_sub_topic: '%s'", map_sub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "map_pub_topic: '%s'", map_pub_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "map_frame: '%s'", map_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "camera_link_optical_frame: '%s'", camera_link_optical_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "track_obstacles: '%s'", (track_ ? "true" : "false"));
    RCLCPP_INFO(this->get_logger(), "obstacle_offset: '%f'", obstacle_offset_);
    RCLCPP_INFO(this->get_logger(), "unoccupied_value: '%d'", MAP_UNOCCUPIED);
    RCLCPP_INFO(this->get_logger(), "occupied_value: '%d'", MAP_OCCUPIED);
    RCLCPP_INFO(this->get_logger(), "unknown_value: '%d'", MAP_UNKNOWN);

    fp_sub_ = this->create_subscription<dynamic_nav_msgs::msg::ObstaclesFootprints>(fp_topic, 10, std::bind(&ObstaclesTracker::footprintsCallback, this, _1));
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(map_sub_topic, 10, std::bind(&ObstaclesTracker::mapCallback, this, _1));
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_pub_topic, 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&ObstaclesTracker::transformCallback, this));
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}


void ObstaclesTracker::footprintsCallback(const dynamic_nav_msgs::msg::ObstaclesFootprints msg) {
    RCLCPP_INFO(this->get_logger(), "got %ld obstacle(s)", msg.count);

    if (!track_ || img_map_.empty()) {
        map_pub_->publish(map_);
        return;
    }

    std::vector<geometry_msgs::msg::Point> map_points;

    // static map's origin coordinates
    tf2::Vector3 origin;
    origin.setX(map_.info.origin.position.x);
    origin.setY(map_.info.origin.position.y);
    origin.setZ(map_.info.origin.position.z);

    for (size_t i = 0; i < msg.points.size(); i++) {
        tf2::Vector3 v(msg.points[i].x, msg.points[i].y, msg.points[i].z); // set points in camera_link_optical frame to tf2::Vector3
        tf2::Vector3 map_pt = (tf_ * v) - origin; // transform points from camera_link_optical to map's origin
        geometry_msgs::msg::Point map_pt_geom;
        map_pt_geom.x = map_pt.x();
        map_pt_geom.y = map_pt.y();
        map_pt_geom.z = map_pt.z();
        map_points.push_back(map_pt_geom);
    }

    float resolution = map_.info.resolution;
    cv::Mat dynamic_img_map = img_map_.clone();

    for (size_t i = 0; i < map_points.size(); i += 4) {
        geometry_msgs::msg::Point pt0 = map_points[i+0];
        geometry_msgs::msg::Point pt3 = map_points[i+3];

        float x = (pt0.x + pt3.x) / 2; // find x_center of obstacle's rectangle in map's origin
        float y = (pt0.y + pt3.y) / 2; // find y_center of obstacle's rectangle in map's origin
        float rx = std::abs(x - pt0.x);
        float ry = std::abs(y - pt0.y);
        uint64_t r = static_cast<int>((std::sqrt(rx * rx + ry * ry) + obstacle_offset_) / resolution); // find obstacle footprint circle radius
        cv::Point center(static_cast<int>(x / resolution), static_cast<int>(y / resolution)); // find circle center
        cv::circle(dynamic_img_map, center, r, {double(IMG_OCCUPIED)}, -1);
    }

    auto dynamic_map = map_;
    for (size_t i = 0; i < size_t(img_map_.rows * img_map_.cols); i++) {
        if (dynamic_img_map.data[i] == IMG_UNOCCUPIED) {
            dynamic_map.data[i] = MAP_UNOCCUPIED;
        }
        if (dynamic_img_map.data[i] == IMG_OCCUPIED) {
            dynamic_map.data[i] = MAP_OCCUPIED;
        }
        if (dynamic_img_map.data[i] == IMG_UNKNOWN) {
            dynamic_map.data[i] = MAP_UNKNOWN;
        }
    }

    map_pub_->publish(dynamic_map);
}


void ObstaclesTracker::mapCallback(const nav_msgs::msg::OccupancyGrid msg) {
    RCLCPP_INFO(this->get_logger(), "static map updated");
    map_ = msg;
    img_map_ = cv::Mat::zeros({int(msg.info.width), int(msg.info.height)}, CV_8UC1);
    for (size_t i = 0; i < size_t(img_map_.rows * img_map_.cols); i++) {
        if (msg.data[i] == MAP_UNOCCUPIED) {
            img_map_.data[i] = IMG_UNOCCUPIED;
        }
        if (msg.data[i] == MAP_OCCUPIED) {
            img_map_.data[i] = IMG_OCCUPIED;
        }
        if (msg.data[i] == MAP_UNKNOWN) {
            img_map_.data[i] = IMG_UNKNOWN;
        }
    }
}


void ObstaclesTracker::transformCallback() {
    try {
        auto transform = tf_buffer_->lookupTransform(map_frame_, camera_link_optical_frame_, tf2::TimePointZero);
        tf2::Quaternion q(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        );
        tf2::Vector3 v(
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        );
        tf_ = tf2::Transform(q, v);
    }
    catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform '%s' to '%s': %s", map_frame_.c_str(), camera_link_optical_frame_.c_str(), ex.what());
    }
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ObstaclesTracker>()); 
    rclcpp::shutdown();
    return 0;
}
