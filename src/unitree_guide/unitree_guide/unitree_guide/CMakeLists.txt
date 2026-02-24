cmake_minimum_required(VERSION 3.14)
project(unitree_guide)


# 添加torch依赖
set(CMAKE_PREFIX_PATH "/home/lenovo/libtorch-cxx11-abi-shared-with-deps-2.3.0+cu118/libtorch" ${CMAKE_PREFIX_PATH})

set(ROBOT_TYPE A1)         # The type of robot, support Go1 and A1 currently
set(PLATFORM amd64)         # The platform to compile, support amd64 and arm64

set(AMENT_MAKE ON)         # Use ament_cmake or not, ON or OFF
set(SIMULATION ON)         # Use Gazebo or not, ON or OFF
set(REAL_ROBOT OFF)          # Link real robot or not, ON or OFF
set(DEBUG OFF)              # Use debug functions or not, ON or OFF
set(MOVE_BASE OFF)               # Need move_base or not, ON or OFF

if(NOT DEFINED ROBOT_TYPE)
    message(FATAL_ERROR "[CMake ERROR] Have not defined ROBOT_TYPE")
endif()
if(NOT DEFINED PLATFORM)
    message(FATAL_ERROR "[CMake ERROR] Have not defined PLATFORM")
endif()

if(${ROBOT_TYPE} STREQUAL "A1")
    add_definitions(-DROBOT_TYPE_A1)
elseif(${ROBOT_TYPE} STREQUAL "Go1")
    add_definitions(-DROBOT_TYPE_Go1)
else()
    message(FATAL_ERROR "[CMake ERROR] The ROBOT_TYPE is error")
endif()

if(((SIMULATION) AND (REAL_ROBOT)) OR ((NOT SIMULATION) AND (NOT REAL_ROBOT)))
    message(FATAL_ERROR "[CMake ERROR] The SIMULATION and REAL_ROBOT can only be one ON one OFF")
endif()

if(SIMULATION OR MOVE_BASE)
    add_definitions(-DRUN_ROS)
    set(AMENT_MAKE ON)
endif()

# 统一使用 C++17（Torch 需要 C++17）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(AMENT_MAKE)
    add_definitions(-DCOMPILE_WITH_ROS)
    if(MOVE_BASE)
        add_definitions(-DCOMPILE_WITH_MOVE_BASE)
    endif()
endif()
# 将 libtorch 库路径嵌入可执行文件，运行时无需手动设置 LD_LIBRARY_PATH
set(CMAKE_INSTALL_RPATH "/home/lenovo/libtorch-cxx11-abi-shared-with-deps-2.3.0+cu118/libtorch/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_LIBRARIES TRUE)

find_package(Torch REQUIRED)
# Force CXX11 ABI=1 to match ROS2 Humble (Torch ships with ABI=0)
string(REPLACE "-D_GLIBCXX_USE_CXX11_ABI=0" "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(TORCH_CXX_FLAGS "")
foreach(_torch_lib torch torch_cpu torch_cuda c10 c10_cuda)
    if(TARGET ${_torch_lib})
        get_target_property(_icd ${_torch_lib} INTERFACE_COMPILE_DEFINITIONS)
        if(_icd)
            list(FILTER _icd EXCLUDE REGEX "GLIBCXX_USE_CXX11_ABI")
            set_target_properties(${_torch_lib} PROPERTIES INTERFACE_COMPILE_DEFINITIONS "${_icd}")
        endif()
        get_target_property(_ico ${_torch_lib} INTERFACE_COMPILE_OPTIONS)
        if(_ico)
            list(FILTER _ico EXCLUDE REGEX "GLIBCXX_USE_CXX11_ABI")
            set_target_properties(${_torch_lib} PROPERTIES INTERFACE_COMPILE_OPTIONS "${_ico}")
        endif()
    endif()
endforeach()
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

if(DEBUG)
    add_definitions(-DCOMPILE_DEBUG)
    find_package(Python2 COMPONENTS Interpreter Development NumPy)
endif()

# ROS2 ament_cmake dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(unitree_legged_msgs REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# LCM 依赖 — 可选，仅在需要时链接
find_package(lcm QUIET)

if(AMENT_MAKE)
    if(SIMULATION)
        add_definitions(-DCOMPILE_WITH_SIMULATION)
        find_package(controller_manager REQUIRED)
        find_package(joint_state_broadcaster REQUIRED)
        find_package(gazebo_ros REQUIRED)
        find_package(gazebo_msgs REQUIRED)
    endif()

    find_package(robot_state_publisher REQUIRED)
endif()

## Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/CustomPoint.msg"
    "msg/CustomMsg.msg"
    DEPENDENCIES builtin_interfaces std_msgs
)

include_directories(
    include
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

if(REAL_ROBOT)
    add_definitions(-DCOMPILE_WITH_REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        include_directories(
            library/unitree_legged_sdk_3.2/include
        )
        link_directories(
            library/unitree_legged_sdk_3.2/lib
        )
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
        include_directories(
            library/unitree_legged_sdk-3.8.0/include
        )
        if(${PLATFORM} STREQUAL "amd64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/amd64
            )
        elseif(${PLATFORM} STREQUAL "arm64")
            link_directories(
                library/unitree_legged_sdk-3.8.0/lib/cpp/arm64
            )
        endif()
    endif()
endif()

# aux_source_directory(src SRC_LIST)
file(GLOB_RECURSE SRC_LIST
    "src/*/*.cpp"
    "src/*/*.cc"
)

if(NOT REAL_ROBOT)
    list(FILTER SRC_LIST EXCLUDE REGEX "IOFREEDOGSDK")
endif()

# CUDA — 自动检测，如果环境变量 CUDA_TOOLKIT_ROOT_DIR 存在则使用
if(DEFINED ENV{CUDA_HOME})
    set(CMAKE_CUDA_COMPILER $ENV{CUDA_HOME}/bin/nvcc)
elseif(EXISTS /usr/local/cuda/bin/nvcc)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
endif()
# C++17 已在上方统一设置，此处不再重复


add_executable(state_from_gazebo src/state_from_gazebo.cpp)
ament_target_dependencies(state_from_gazebo
    rclcpp
    nav_msgs
    geometry_msgs
    tf2
    tf2_ros
    gazebo_msgs
    sensor_msgs
)

add_executable(junior_ctrl src/main.cpp ${SRC_LIST})
if(AMENT_MAKE)
    ament_target_dependencies(junior_ctrl
        rclcpp
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
        tf2
        tf2_ros
        unitree_legged_msgs
        rosgraph_msgs
    )
    if(SIMULATION)
        ament_target_dependencies(junior_ctrl
            gazebo_ros
            gazebo_msgs
        )
    endif()
endif()
target_link_libraries(junior_ctrl ${TORCH_LIBRARIES})
if(DEBUG)
    target_include_directories(junior_ctrl PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
    target_link_libraries(junior_ctrl Python2::Python Python2::NumPy)
endif()
if(REAL_ROBOT)
    if(${ROBOT_TYPE} STREQUAL "A1")
        if(${PLATFORM} STREQUAL "amd64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_amd64.so)
        elseif(${PLATFORM} STREQUAL "arm64")
            target_link_libraries(junior_ctrl libunitree_legged_sdk_arm64.so)
        endif()
    elseif(${ROBOT_TYPE} STREQUAL "Go1")
            target_link_libraries(junior_ctrl libunitree_legged_sdk.a)
    endif()
endif()
target_link_libraries(junior_ctrl -pthread)
if(lcm_FOUND)
    target_link_libraries(junior_ctrl lcm)
endif()

# Install executables
install(TARGETS
    junior_ctrl
    state_from_gazebo
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
    scripts/virtual_joy.py
    scripts/pointcloud2livox.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY include/
    DESTINATION include
)

# Install launch, config, etc. if they exist
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()
