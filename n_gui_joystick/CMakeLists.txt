cmake_minimum_required(VERSION 2.8.3)
project(n_gui_joystick)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
  cv_bridge
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenGL_INCLUDE_DIRS}
)
add_executable(n_gui_joystick_node
   src/gl_main.cpp
   src/geometory.cpp
   src/viewgui.cpp
)

add_dependencies(n_gui_joystick_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(n_gui_joystick_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${OpenGL_LIBRARIES}
  ${GLUT_LIBRARY}
  -lglut -lGLU -lGL -lm -lSDL
 )


