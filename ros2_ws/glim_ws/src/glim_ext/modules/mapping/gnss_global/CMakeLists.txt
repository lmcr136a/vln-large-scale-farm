cmake_minimum_required(VERSION 3.5.2)
project(gnss_global)

set(CMAKE_CXX_STANDARD 17)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(gnss_global SHARED
  src/glim_ext/gnss_global_module_ros2.cpp
  src/glim_ext/geodetic.cpp
)

ament_target_dependencies(gnss_global
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
)

target_include_directories(gnss_global PRIVATE
  include
  ${GTSAM_INCLUDE_DIRS}
)

target_link_libraries(gnss_global
  glim_ext
  ${GTSAM_LIBRARIES}
  spdlog::spdlog
)

install(FILES config/config_gnss_global.json
  DESTINATION share/glim_ext/config
)
