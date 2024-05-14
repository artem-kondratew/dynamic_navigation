cmake_minimum_required(VERSION 3.8)
project(dynamic_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
    launch config
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
