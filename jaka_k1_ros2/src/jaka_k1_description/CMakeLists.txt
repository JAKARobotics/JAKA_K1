cmake_minimum_required(VERSION 3.8)

project(jaka_k1_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

install(DIRECTORY config launch k1-arm k1-de DESTINATION share/${PROJECT_NAME})


ament_package()