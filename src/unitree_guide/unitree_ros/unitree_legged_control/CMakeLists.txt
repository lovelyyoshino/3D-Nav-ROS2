cmake_minimum_required(VERSION 3.5)
project(unitree_legged_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(control_toolbox REQUIRED)

add_library(unitree_legged_control SHARED
    src/joint_controller.cpp
    src/unitree_joint_control_tool.cpp
)
target_include_directories(unitree_legged_control PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(unitree_legged_control
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    realtime_tools
    unitree_legged_msgs
    geometry_msgs
    urdf
    control_toolbox
)

pluginlib_export_plugin_description_file(controller_interface unitree_controller_plugins.xml)

install(TARGETS unitree_legged_control
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/
    DESTINATION include
)

ament_package()
