cmake_minimum_required(VERSION 3.8)
project(dynamic_navigation)

find_package(ament_cmake REQUIRED)
find_package(dynamic_nav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(dynamic_controller
    src/dynamic_controller.cpp
)

ament_target_dependencies(dynamic_controller
    dynamic_nav_interfaces
    geometry_msgs
    nav2_msgs
    rclcpp
    rclcpp_action
    std_msgs
)

install(DIRECTORY
    src launch config scripts
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    dynamic_controller
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
