cmake_minimum_required(VERSION 3.5)
project(example_package)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# Declare executable
add_executable(filter_node src/filter_node.cpp)
ament_target_dependencies(filter_node
  rclcpp
  sensor_msgs
  std_msgs
  geometry_msgs
  nav_msgs
)

target_link_libraries(filter_node ${OpenCV_LIBS})

# Install
install(TARGETS
  filter_node
  DESTINATION lib/${PROJECT_NAME}
)

# Package
ament_package()

