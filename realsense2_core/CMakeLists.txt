cmake_minimum_required(VERSION 3.5)
project(realsense2_core)


add_compile_options(-std=c++11)

option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

set(msg_files
    msg/IMUInfo.msg
    msg/Extrinsics.msg
    )

# Actually generate the language-specific message and service files
rosidl_generate_interfaces(realsense_camera
    ${msg_files}
    DEPENDENCIES builtin_interfaces sensor_msgs std_msgs
)



ament_export_dependencies(${PROJECT_NAME})
ament_export_dependencies(
    rosidl_default_runtime
    std_msgs
    geometry_msgs
)

ament_package()

