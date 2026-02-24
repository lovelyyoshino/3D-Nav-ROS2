cmake_minimum_required(VERSION 3.5)
project(a1_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config meshes urdf xacro
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()
