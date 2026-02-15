cmake_minimum_required(VERSION 3.8)
project(pavbot_gps_nav)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(gps_waypoint_follower src/gps_waypoint_follower.cpp)
ament_target_dependencies(gps_waypoint_follower rclcpp sensor_msgs std_msgs geometry_msgs)

install(TARGETS gps_waypoint_follower DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
