cmake_minimum_required(VERSION 3.0.2)
project(rover_base)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roslaunch
  roscpp
)

catkin_package(
  CATKIN_DEPENDS 
    controller_manager 
    hardware_interface 
    roscpp)

include_directories(
${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_node 
  src/rover_base.cpp)

target_link_libraries(${PROJECT_NAME}_node 
  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

