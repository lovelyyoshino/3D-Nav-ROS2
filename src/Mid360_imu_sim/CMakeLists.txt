cmake_minimum_required(VERSION 3.8)
project(livox_laser_simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(livox_laser_simulation SHARED
  src/livox_points_plugin.cpp
  src/livox_ode_multiray_shape.cpp
)

ament_target_dependencies(livox_laser_simulation
  rclcpp
  sensor_msgs
  tf2_ros
  tf2
  geometry_msgs
  gazebo_ros
  gazebo_dev
)

target_link_libraries(livox_laser_simulation
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

install(TARGETS livox_laser_simulation
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch scan_mode urdf worlds rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
