cmake_minimum_required(VERSION 3.5)
project(realsense_ros2)

set(CMAKE_CXX_STANDARD 11)

# if(NOT CMAKE_CXX_STANDARD)
#   set(CMAKE_CXX_STANDARD 14)
# endif()
# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wunused-parameter -Wdeprecated-declarations -Wsign-compare)
# endif()

## Find catkin macros and libraries

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV  REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(realsense REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(PCL 1.11 REQUIRED)


###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Declare a C++ executable
add_executable(
	${PROJECT_NAME}_node
		src/main.cpp
		src/sensor_node.cpp
		src/base_realsense_node.cpp
)
add_definitions(${PCL_DEFINITIONS})
## Specify libraries to link a library or executable target against
ament_target_dependencies(
	${PROJECT_NAME}_node
	rclcpp
	std_msgs
	sensor_msgs
    tf2
    tf2_ros
	geometry_msgs
	Eigen3
	OpenCV
	rclcpp_action 
	realsense
	image_transport
	cv_bridge
	PCL
 )
target_include_directories(${PROJECT_NAME}_node PRIVATE include )
target_link_libraries(${PROJECT_NAME}_node  ${OpenCV_LIBS}  ${PCL_LIBRARIES} realsense boost_system )

ament_export_include_directories(include)
ament_export_dependencies(${PROJECT_NAME} )

install(TARGETS
${PROJECT_NAME}_node 
DESTINATION lib/${PROJECT_NAME}
)

ament_package()
