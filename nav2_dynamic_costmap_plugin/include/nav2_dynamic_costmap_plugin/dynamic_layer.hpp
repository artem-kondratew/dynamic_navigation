#ifndef NAV2_DYNAMIC_COSTMAP_PLUGIN_DYNAMIC_LAYER_HPP
#define NAV2_DYNAMIC_COSTMAP_PLUGIN_DYNAMIC_LAYER_HPP


#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv4/opencv2/highgui.hpp>

#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>

#include "dynamic_nav_interfaces/msg/dynamic_layer_msg.hpp"


namespace nav2_dynamic_costmap_plugin {

class DynamicLayer : public nav2_costmap_2d::Layer {
private:

    rclcpp::Subscription<dynamic_nav_interfaces::msg::DynamicLayerMsg>::SharedPtr sub_;

    bool need_recalculation_; // Indicates that the entire gradient should be recalculated next time

    dynamic_nav_interfaces::msg::DynamicLayerMsg obstacles_;
    cv::Mat img_;

    double last_min_x_, last_min_y_, last_max_x_, last_max_y_;

public:
    DynamicLayer();

    virtual void onInitialize();
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x, double* max_y);
    virtual void updateCosts(nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j);
    virtual void reset() {return;}
    virtual void onFootprintChanged();
    virtual bool isClearable() {return false;}

private:
    void callback(const dynamic_nav_interfaces::msg::DynamicLayerMsg msg);
};

}

#endif // NAV2_DYNAMIC_COSTMAP_PLUGIN_DYNAMIC_LAYER_HPP
