cmake_minimum_required(VERSION 3.8)
project(filters)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

# EKF node
add_executable(ekf_node
  src/EKF_node.cpp
  src/EKF.cpp
)

ament_target_dependencies(ekf_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  Eigen3
  tf2_ros  
  visualization_msgs
  tf2_geometry_msgs
)

target_include_directories(ekf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

# add_executable(kalman_node src/kalman_node.cpp)
# ament_target_dependencies(kalman_node
#   rclcpp
#   nav_msgs
#   sensor_msgs
#   geometry_msgs
#   std_msgs
#   Eigen3
#   tf2_ros  
#   visualization_msgs
#   tf2_geometry_msgs
# )

#GroundTruth
add_executable(ground_truth_checker 
  src/ground_truth_checker.cpp
)

ament_target_dependencies(ground_truth_checker 
  rclcpp 
  nav_msgs 
  gazebo_msgs
)

# KF node
add_executable(kf_node
  src/KF_node.cpp
  src/KF.cpp
)

ament_target_dependencies(kf_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  Eigen3
  tf2_ros  
  visualization_msgs
  tf2_geometry_msgs
)

target_include_directories(kf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)


# PF node
add_executable(pf_node
  src/PF_node.cpp
  src/PF.cpp
)

ament_target_dependencies(pf_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  std_msgs
  Eigen3
  tf2_ros  
  visualization_msgs
  tf2_geometry_msgs
)

target_include_directories(pf_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  ${EIGEN3_INCLUDE_DIRS}
)

# ==========================================

install(TARGETS
  ekf_node
  kf_node
  pf_node      
  ground_truth_checker 
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
