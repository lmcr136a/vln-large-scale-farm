cmake_minimum_required(VERSION 3.8)
project(glim_converters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

add_executable(glim_converters src/glim_converters.cpp)
target_include_directories(glim_converters PUBLIC include)
ament_target_dependencies(glim_converters
  rclcpp
  sensor_msgs
  geometry_msgs
  livox_ros_driver2
)

install(TARGETS glim_converters
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

