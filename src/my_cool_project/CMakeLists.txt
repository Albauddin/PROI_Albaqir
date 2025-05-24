cmake_minimum_required(VERSION 3.5)
project(my_cool_project)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)


# Add include directories if needed
# include_directories(include)

# Add executable(s)
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp std_msgs sensor_msgs geometry_msgs nav_msgs)

# Install binaries
# install(TARGETS
#   my_node
#   DESTINATION lib/${PROJECT_NAME}
# )

# Install launch, maps, and param files
install(DIRECTORY
  launch
  maps
  param
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Finalize the package
ament_package()
