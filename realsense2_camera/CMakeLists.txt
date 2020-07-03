cmake_minimum_required(VERSION 3.5)
project(realsense2_camera)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-std=c++11)

option(BUILD_WITH_OPENMP "Use OpenMP" OFF)
option(SET_USER_BREAK_AT_STARTUP "Set user wait point in startup (for debug)" OFF)

## FIX LATER
set(realsense2_core_FOUND 1)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(realsense2_core REQUIRED)
# find_package(ddynamic_reconfigure REQUIRED)
# find_package(diagnostic_updater REQUIRED)
find_package(realsense REQUIRED)



if(BUILD_WITH_OPENMP)
    find_package(OpenMP)
    if(NOT OpenMP_FOUND)
        message(FATAL_ERROR "\n\n OpenMP is missing!\n\n")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    endif()
endif()

if(SET_USER_BREAK_AT_STARTUP)
	message("GOT FLAG IN CmakeLists.txt")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBPDEBUG")
endif()


if(NOT realsense_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 1.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

string(TOUPPER "${CMAKE_BUILD_TYPE}" uppercase_CMAKE_BUILD_TYPE)
if (${uppercase_CMAKE_BUILD_TYPE} STREQUAL "RELEASE")
    message(STATUS "Create Release Build.")
    set(CMAKE_CXX_FLAGS "-O2 ${CMAKE_CXX_FLAGS}")
else()
    message(STATUS "Create Debug Build.")
endif()

set(CMAKE_CXX_FLAGS "-fPIE -fPIC -std=c++11 -D_FORTIFY_SOURCE=2 -fstack-protector -Wformat -Wformat-security -Wall ${CMAKE_CXX_FLAGS}")


set(CMAKE_NO_SYSTEM_FROM_IMPORTED true)

include_directories(
    include
    ${realsense_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    )

# RealSense ROS Node
add_library(${PROJECT_NAME}
    include/constants.hpp
    include/realsense_node_factory.hpp
    include/base_realsense_node.hpp
    include/t265_realsense_node.hpp
    src/realsense_node_factory.cpp
    src/base_realsense_node.cpp
    src/t265_realsense_node.cpp
    )

ament_target_dependencies(${PROJECT_NAME}
#   ${PROJECT_NAME}_generate_messages_cpp
#     message_runtime  
    realsense2_core
    sensor_msgs std_msgs
#     nodelet
    tf2
    cv_bridge
    image_transport
    nav_msgs
)


target_include_directories(${PROJECT_NAME} PRIVATE ${realsense_INCLUDE_DIR})

target_link_libraries(${PROJECT_NAME}   ${realsense_LIBRARY}    ${CMAKE_THREAD_LIBS_INIT}   )

# Install nodelet library
# install(TARGETS ${PROJECT_NAME}
#     ARCHIVE DESTINATIONshare/${PROJECT_NAME}
#     LIBRARY DESTINATION share/${PROJECT_NAME}
#     RUNTIME DESTINATION share/${PROJECT_NAME}
#     )

# Install header files
install(DIRECTORY include/
    DESTINATION share/${PROJECT_NAME}
    )

# Install launch files
install(DIRECTORY launch/
    DESTINATION share/${PROJECT_NAME}
    )

# Install rviz files
install(DIRECTORY rviz/
    DESTINATION share/${PROJECT_NAME}
    )

# Install xml files
install(FILES nodelet_plugins.xml
    DESTINATION share/${PROJECT_NAME}
    )


ament_export_dependencies(${PROJECT_NAME})
ament_export_dependencies(
    rosidl_default_runtime
    std_msgs
    geometry_msgs
)

ament_package()

