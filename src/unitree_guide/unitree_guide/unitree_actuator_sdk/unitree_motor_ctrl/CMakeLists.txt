cmake_minimum_required(VERSION 3.5)
project(unitree_motor_ctrl)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include/unitree_motor_ctrl
)

link_directories(
  lib
)

add_executable(${PROJECT_NAME}_node src/check.cpp)
target_link_libraries(${PROJECT_NAME}_node
  Unitree_motor_SDK_Linux
)
ament_target_dependencies(${PROJECT_NAME}_node rclcpp)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
