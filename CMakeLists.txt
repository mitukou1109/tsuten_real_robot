cmake_minimum_required(VERSION 3.0.2)
project(tsuten_real_robot)

find_package(catkin REQUIRED
  COMPONENTS
    nav_msgs
    roscpp
    tf2
    tf2_ros
    tsuten_msgs
)

catkin_package(
  CATKIN_DEPENDS
    nav_msgs
    roscpp
    tf2
    tf2_ros
    tsuten_msgs
)

add_executable(odom_converter
  src/odom_converter.cpp
)
target_include_directories(odom_converter
  PUBLIC
    ${catkin_INCLUDE_DIRS}
)
target_link_libraries(odom_converter
  ${catkin_LIBRARIES}
)