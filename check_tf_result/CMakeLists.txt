cmake_minimum_required(VERSION 2.8.3)
project(check_tf_result)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp 
	std_msgs 
	sensor_msgs 
	geometry_msgs 
	nav_msgs 
	tf  
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
##  ${OpenCV_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
)
add_executable(check_tf_result_node 
   src/ros_main.cpp
)

add_dependencies(check_tf_result_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(check_tf_result_node 
  ${catkin_LIBRARIES}
 )


