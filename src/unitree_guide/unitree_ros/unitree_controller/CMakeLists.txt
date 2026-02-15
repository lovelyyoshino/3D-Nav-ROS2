cmake_minimum_required(VERSION 3.5)
project(unitree_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# Declare a C++ library
add_library(${PROJECT_NAME}
    src/body.cpp
)
ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    unitree_legged_msgs
)

add_executable(unitree_external_force src/external_force.cpp)
ament_target_dependencies(unitree_external_force
    rclcpp
    geometry_msgs
)

add_executable(unitree_servo src/servo.cpp)
target_link_libraries(unitree_servo ${PROJECT_NAME})
ament_target_dependencies(unitree_servo
    rclcpp
    std_msgs
    geometry_msgs
    sensor_msgs
    nav_msgs
    unitree_legged_msgs
)

add_executable(unitree_move_kinetic src/move_publisher.cpp)
ament_target_dependencies(unitree_move_kinetic
    rclcpp
    gazebo_msgs
    tf2
    tf2_geometry_msgs
)

# Install targets
install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS
    unitree_external_force
    unitree_servo
    unitree_move_kinetic
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
    DESTINATION include
)

ament_package()
