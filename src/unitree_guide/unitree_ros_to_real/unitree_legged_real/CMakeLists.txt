cmake_minimum_required(VERSION 3.8)
project(unitree_legged_real)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -fPIC")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(unitree_legged_sdk REQUIRED)

message("-- CMAKE_SYSTEM_PROCESSOR: ${CMAKE_SYSTEM_PROCESSOR}")
if("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64.*")
  set(ARCH amd64)
else()
  set(ARCH arm64)
endif()

set(EXTRA_LIBS -pthread ${unitree_legged_sdk_LIBRARIES})

include_directories(
    include
    ${unitree_legged_sdk_INCLUDE_DIRS}
)

add_executable(ros_example_walk src/exe/example_walk.cpp)
ament_target_dependencies(ros_example_walk rclcpp unitree_legged_msgs unitree_legged_sdk)
target_link_libraries(ros_example_walk ${EXTRA_LIBS})

add_executable(ros_example_position src/exe/example_position.cpp)
ament_target_dependencies(ros_example_position rclcpp unitree_legged_msgs unitree_legged_sdk)
target_link_libraries(ros_example_position ${EXTRA_LIBS})

add_executable(state_sub src/exe/state_sub.cpp)
ament_target_dependencies(state_sub rclcpp unitree_legged_msgs unitree_legged_sdk)
target_link_libraries(state_sub ${EXTRA_LIBS})

add_executable(ros_udp src/exe/ros_udp.cpp)
ament_target_dependencies(ros_udp rclcpp geometry_msgs unitree_legged_msgs unitree_legged_sdk)
target_link_libraries(ros_udp ${EXTRA_LIBS})

add_executable(control_via_keyboard src/exe/control_via_keyboard.cpp)
ament_target_dependencies(control_via_keyboard rclcpp geometry_msgs)
target_link_libraries(control_via_keyboard ${EXTRA_LIBS})

add_executable(twist_sub src/exe/twist_sub.cpp)
ament_target_dependencies(twist_sub rclcpp geometry_msgs unitree_legged_msgs unitree_legged_sdk)
target_link_libraries(twist_sub ${EXTRA_LIBS})

install(TARGETS
  ros_example_walk ros_example_position state_sub ros_udp control_via_keyboard twist_sub
  DESTINATION lib/${PROJECT_NAME})

ament_package()

