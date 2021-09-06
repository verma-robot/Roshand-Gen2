cmake_minimum_required(VERSION 3.0.2)
project(roshand_gen2_visualizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roshand_gen2_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(  ${catkin_INCLUDE_DIRS})

