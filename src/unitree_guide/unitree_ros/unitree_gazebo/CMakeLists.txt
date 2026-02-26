cmake_minimum_required(VERSION 3.5)
project(unitree_gazebo)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(gazebo REQUIRED)
find_package(controller_manager REQUIRED)
find_package(robot_state_publisher REQUIRED)

include_directories(
    ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(unitreeFootContactPlugin SHARED plugin/foot_contact_plugin.cc)
ament_target_dependencies(unitreeFootContactPlugin
    rclcpp
    gazebo_ros
    geometry_msgs
)
target_link_libraries(unitreeFootContactPlugin ${GAZEBO_LIBRARIES})

add_library(unitreeDrawForcePlugin SHARED plugin/draw_force_plugin.cc)
ament_target_dependencies(unitreeDrawForcePlugin
    rclcpp
    gazebo_ros
    geometry_msgs
)
target_link_libraries(unitreeDrawForcePlugin ${GAZEBO_LIBRARIES})

install(TARGETS
    unitreeFootContactPlugin
    unitreeDrawForcePlugin
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY launch worlds models config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()