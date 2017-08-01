cmake_minimum_required(VERSION 2.8.3)
project(leap_motion_controller)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  geometry_msgs
  visualization_msgs
)

add_definitions(-std=c++11)

catkin_package(
# INCLUDE_DIRS include
# LIBRARIES leap_motion_controller
 CATKIN_DEPENDS std_msgs geometry_msgs visualization_msgs 
# DEPENDS system_lib
)

link_directories(lib)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${LEAP_SDK}/include/
)

add_executable(leap_motion src/leap_motion.cpp)

add_dependencies(leap_motion leap_motion_controller_gencpp)

target_link_libraries(leap_motion ${catkin_LIBRARIES} libLeap.so)

