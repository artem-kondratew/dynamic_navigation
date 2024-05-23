cmake_minimum_required(VERSION 3.5)
project(nav2_dynamic_costmap_plugin)

set(lib_name ${PROJECT_NAME}_core)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(dynamic_nav_interfaces REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    pluginlib
    dynamic_nav_interfaces)

add_library(${lib_name} SHARED
    src/dynamic_layer.cpp)

target_link_libraries(${lib_name} ${OpenCV_LIBS})

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})

install(TARGETS ${lib_name}
        DESTINATION lib)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs gradient_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d dynamic_layer.xml)
ament_target_dependencies(${lib_name} ${dep_pkgs})
ament_package()