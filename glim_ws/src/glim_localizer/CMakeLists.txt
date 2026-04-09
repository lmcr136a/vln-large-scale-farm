cmake_minimum_required(VERSION 3.16)
project(glim_localizer)

set(CMAKE_CXX_STANDARD 17)
add_compile_options(-O3)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)
find_package(glim REQUIRED)

# ImGui / guik / glk — init_pose_viewer extension module용
find_package(imgui QUIET)
find_package(guik QUIET)
find_package(glk QUIET)

# ──────────────────────────────────────────────
# 1. map_loader_node
# ──────────────────────────────────────────────
add_executable(map_loader_node src/map_loader_node.cpp)
target_include_directories(map_loader_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
)
ament_target_dependencies(map_loader_node
  rclcpp sensor_msgs geometry_msgs pcl_conversions
)
target_link_libraries(map_loader_node
  glim::glim
  ${PCL_LIBRARIES}
  Boost::filesystem
)

# ──────────────────────────────────────────────
# 2. localizer_ext  (GLIM extension module — submap callback 직접 수신)
# ──────────────────────────────────────────────
add_library(localizer_ext SHARED src/localizer_ext.cpp)
target_include_directories(localizer_ext PRIVATE
  include
  /usr/local/include
)
ament_target_dependencies(localizer_ext
  rclcpp geometry_msgs
)
target_link_libraries(localizer_ext
  glim::glim
  Boost::filesystem
)

# ──────────────────────────────────────────────
# 4. init_pose_viewer  (GLIM extension module .so)
# ──────────────────────────────────────────────
add_library(init_pose_viewer SHARED src/init_pose_viewer.cpp)
target_include_directories(init_pose_viewer PRIVATE
  include
  /usr/local/include
)
ament_target_dependencies(init_pose_viewer
  rclcpp geometry_msgs
)
target_link_libraries(init_pose_viewer
  glim::glim
  ${PCL_LIBRARIES}
)

# ──────────────────────────────────────────────
# Install
# ──────────────────────────────────────────────
install(TARGETS
  map_loader_node localizer_ext init_pose_viewer
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

ament_package()