#include <chrono>
#include <iostream>
#include <memory>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "dynamic_nav_interfaces/msg/min_distance.hpp"


using namespace std::chrono_literals;


class DynamicController : public rclcpp::Node {
private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
    rclcpp::Subscription<dynamic_nav_interfaces::msg::MinDistance>::SharedPtr min_distance_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr danger_distance_pub_;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr cancel_navigation_client_;

    rclcpp::TimerBase::SharedPtr timer_;

    bool verbose_;
    bool use_emergency_stop_;
    float danger_distance_;
    float danger_angle_;
    bool allow_driving_;
    geometry_msgs::msg::PoseStamped goal_pose_;
    size_t wait_time_;
    float backward_linear_vel_;
    bool check_min_dist_;
    bool start_check_min_dist_;
    bool first_goal_pose_;

public:
    DynamicController();

private:
    void delay(size_t ms);
    void cmdVelCallback(geometry_msgs::msg::Twist msg);
    void goalPoseCallback(geometry_msgs::msg::PoseStamped msg);
    void minDistanceCallback(const dynamic_nav_interfaces::msg::MinDistance msg);
    void cancelNavigation();
    void waitObstacleMoving();
    void moveBackward();
    void minDistanceChecker();
    void recovery();
};


DynamicController::DynamicController() : Node("dynamic_controller") {
    using std::placeholders::_1;

    this->declare_parameter("verbose", true);
    this->declare_parameter("goal_pose_topic", "");
    this->declare_parameter("nav2_cmd_vel_topic", "");
    this->declare_parameter("cmd_vel_topic", "");
    this->declare_parameter("min_distance_topic", "");
    this->declare_parameter("danger_distance_topic", "");
    this->declare_parameter("use_emergency_stop", false);
    this->declare_parameter("danger_distance", 0.0);
    this->declare_parameter("danger_angle", 0.0);
    this->declare_parameter("wait_time", 0);
    this->declare_parameter("backward_linear_vel", 0.0);

    verbose_ = this->get_parameter("verbose").as_bool();
    std::string goal_pose_topic = this->get_parameter("goal_pose_topic").as_string();
    std::string nav2_cmd_vel_topic = this->get_parameter("nav2_cmd_vel_topic").as_string();
    std::string cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();
    std::string min_distance_topic = this->get_parameter("min_distance_topic").as_string();
    std::string danger_distance_topic = this->get_parameter("danger_distance_topic").as_string();
    use_emergency_stop_ = this->get_parameter("use_emergency_stop").as_bool();
    danger_distance_ = this->get_parameter("danger_distance").as_double();
    danger_angle_ = this->get_parameter("danger_angle").as_double();
    wait_time_ = this->get_parameter("wait_time").as_int();
    backward_linear_vel_ = this->get_parameter("backward_linear_vel").as_double();

    RCLCPP_INFO(this->get_logger(), "verbose: '%s'", verbose_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "goal_pose_topic: '%s'", goal_pose_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "nav2_cmd_vel_topic: '%s'", nav2_cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "cmd_vel_topic: '%s'", cmd_vel_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "min_distance_topic: '%s'", min_distance_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "danger_distance_topic: '%s'", danger_distance_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "use_emergency_stop: '%s'", use_emergency_stop_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "danger_distance: '%f'", danger_distance_);
    RCLCPP_INFO(this->get_logger(), "danger_angle: '%f'", danger_angle_);
    RCLCPP_INFO(this->get_logger(), "wait_time: '%ld'", wait_time_);
    RCLCPP_INFO(this->get_logger(), "backward_linear_vel: '%f'", backward_linear_vel_);

    goal_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10, std::bind(&DynamicController::goalPoseCallback, this, _1));
    goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 10);
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(nav2_cmd_vel_topic, 10, std::bind(&DynamicController::cmdVelCallback, this, _1));
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    min_distance_sub_ = this->create_subscription<dynamic_nav_interfaces::msg::MinDistance>(min_distance_topic, 1, std::bind(&DynamicController::minDistanceCallback, this, _1));
    danger_distance_pub_ = this->create_publisher<std_msgs::msg::Bool>(danger_distance_topic, 10);


    cancel_navigation_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this,"/navigate_to_pose");

    timer_ = this->create_wall_timer(10ms, std::bind(&DynamicController::minDistanceChecker, this));

    allow_driving_ = true;
    check_min_dist_ = false;
    start_check_min_dist_ = false;
    first_goal_pose_ = false;
}


void DynamicController::delay(size_t ms) {
    auto start = std::chrono::system_clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start).count() < ms) {}
}


void DynamicController::cmdVelCallback(geometry_msgs::msg::Twist msg) {
    if (use_emergency_stop_ && !allow_driving_) {
        RCLCPP_WARN(this->get_logger(), "driving is forbidden");
        return;
    }
    cmd_vel_pub_->publish(msg);
}


void DynamicController::goalPoseCallback(geometry_msgs::msg::PoseStamped msg) {
    goal_pose_ = msg;
    first_goal_pose_ = true;
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "goal_pose updated");
    }
}


void DynamicController::minDistanceCallback(const dynamic_nav_interfaces::msg::MinDistance msg) {
    static std_msgs::msg::Bool danger_msg, not_danger_msg;
    danger_msg.data = true;
    not_danger_msg.data = false;

    if (check_min_dist_ || start_check_min_dist_) {
        return;
    }

    float distance = msg.min_distance;
    float angle = std::abs(msg.angle / 3.14 * 180);
    if (verbose_) {
        RCLCPP_INFO(this->get_logger(), "min_distance = %f; angle = %f; allow_driving = %s", distance, angle, allow_driving_ ? "true" : "false");
    }

    if (!use_emergency_stop_ || distance > danger_distance_ || angle > danger_angle_) {
        allow_driving_ = true;
        danger_distance_pub_->publish(not_danger_msg);
        return;
    }

    if (allow_driving_) {
        danger_distance_pub_->publish(danger_msg);
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        allow_driving_ = false;
    }

    recovery();
}


void DynamicController::cancelNavigation() {
    if (verbose_) {
        if (!first_goal_pose_) {
            RCLCPP_INFO(this->get_logger(), "no goal_pose to cancel");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "goal_pose canceled");
        cancel_navigation_client_->async_cancel_all_goals();
    }
}


void DynamicController::waitObstacleMoving() {
    RCLCPP_INFO(this->get_logger(), "start waiting obstacle moving");
    delay(wait_time_);
    RCLCPP_INFO(this->get_logger(), "stop waiting obstacle moving");
}


void DynamicController::moveBackward() {
    auto v = geometry_msgs::msg::Twist();
    v.linear.x = backward_linear_vel_;
    RCLCPP_INFO(this->get_logger(), "start moving backward");
    cmd_vel_pub_->publish(v);
    delay(2000);
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
    RCLCPP_INFO(this->get_logger(), "stop moving backward");
}


void DynamicController::minDistanceChecker() {
    static std::chrono::_V2::system_clock::time_point min_dist_start_time;
    size_t delay = 1000;
    if (start_check_min_dist_) {
        check_min_dist_ = true;
        start_check_min_dist_ = false;
        min_dist_start_time = std::chrono::system_clock::now();
        return;
    }
    if (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - min_dist_start_time).count() > delay) {
        check_min_dist_ = false;
    }
}


void DynamicController::recovery() {
    RCLCPP_INFO(this->get_logger(), "start recovery");

    cancelNavigation();
    // waitObstacleMoving();
    moveBackward();
    start_check_min_dist_ = true;

    if (first_goal_pose_) {
        RCLCPP_INFO(this->get_logger(), "send goal_pose");
        goal_pose_pub_->publish(goal_pose_);
    }
    else {
        RCLCPP_INFO(this->get_logger(), "no goal_pose to send");
    }
    
    RCLCPP_INFO(this->get_logger(), "stop recovery");
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicController>()); 
    rclcpp::shutdown();
    return 0;
}
