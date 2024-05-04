#include "nav2_dynamic_costmap_plugin/dynamic_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"


using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;


namespace nav2_dynamic_costmap_plugin {

DynamicLayer::DynamicLayer()
: last_min_x_(-std::numeric_limits<float>::max()),
  last_min_y_(-std::numeric_limits<float>::max()),
  last_max_x_(std::numeric_limits<float>::max()),
  last_max_y_(std::numeric_limits<float>::max()) {/* PASS */}


// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void DynamicLayer::onInitialize() {
    auto node = node_.lock(); 
    declareParameter("enabled", rclcpp::ParameterValue(true));
    node->get_parameter(name_ + "." + "enabled", enabled_);

    need_recalculation_ = true;
    current_ = true;

    sub_ = node->create_subscription<dynamic_nav_msgs::msg::DynamicLayerMsg>(
        "/dynamic_detector/local_costmap/obstacles",
        10,
        std::bind(&DynamicLayer::callback, this, std::placeholders::_1)
    );
}


// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void DynamicLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x,
  double * min_y, double * max_x, double * max_y) {
  if (need_recalculation_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    // For some reason when I make these -<double>::max() it does not
    // work with Costmap2D::worldToMapEnforceBounds(), so I'm using
    // -<float>::max() instead.
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recalculation_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = std::min(tmp_min_x, *min_x);
    *min_y = std::min(tmp_min_y, *min_y);
    *max_x = std::max(tmp_max_x, *max_x);
    *max_y = std::max(tmp_max_y, *max_y);
  }
}


void DynamicLayer::onFootprintChanged() {
    need_recalculation_ = true;
}


// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void DynamicLayer::updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/) {
    if (!enabled_) {
        return;
    }

    unsigned int size_x = master_grid.getSizeInCellsX();
    unsigned int size_y = master_grid.getSizeInCellsY();

    cv::Mat map = cv::Mat::zeros({int(size_x), int(size_y)}, CV_8UC1);
    std::memcpy(map.data, master_grid.getCharMap(), sizeof(uint8_t) * size_x * size_y);

    if (obstacles_.obstacles.size() == 0) {
        img_ = map;
        std::memcpy(img_.data, master_grid.getCharMap(), sizeof(uint8_t) * size_x * size_y);
        cv::flip(img_, img_, 1);
        cv::imshow("plugin", img_);
        cv::waitKey(1);
        return;
    }

    // local_costmap's origin coordinates
    tf2::Vector3 origin;
    origin.setX(master_grid.getOriginX());
    origin.setY(master_grid.getOriginY());
    origin.setZ(0);

    float resolution = master_grid.getResolution();

    for (size_t i = 0; i < obstacles_.obstacles.size(); i++) {
        tf2::Vector3 center;
        center.setX(obstacles_.obstacles[i].x);
        center.setY(obstacles_.obstacles[i].y);
        center.setZ(0);

        auto relative_center = center - origin;
        RCLCPP_INFO(
            rclcpp::get_logger("dynamic_layer"),
            "get %ld obstacles: %lf %lf     %lf %lf",
            obstacles_.obstacles.size(),
            center.x(), center.y(),
            relative_center.x(), relative_center.y()
        );

        int cx = static_cast<int>(relative_center.x() / resolution);
        int cy = static_cast<int>(relative_center.y() / resolution);

        int r = static_cast<int>(obstacles_.obstacles[i].r / resolution);

        cv::ellipse(map, {cx, cy}, {r, r}, 0, 0, 360, {LETHAL_OBSTACLE}, cv::FILLED);
    }

    std::memcpy(master_grid.getCharMap(), map.data, sizeof(uint8_t) * size_x * size_y);

    cv::flip(map, img_, 1);
    cv::imshow("plugin", img_);
    cv::waitKey(1);
}


void DynamicLayer::callback(dynamic_nav_msgs::msg::DynamicLayerMsg msg) {
    // RCLCPP_INFO(rclcpp::get_logger("dynamic_layer"), "get %ld obstacle(s)", msg.count);
    obstacles_ = msg;
}

}


// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_dynamic_costmap_plugin::DynamicLayer, nav2_costmap_2d::Layer)
