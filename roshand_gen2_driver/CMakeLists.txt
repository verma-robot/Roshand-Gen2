cmake_minimum_required(VERSION 3.0.2)
project(roshand_gen2_driver)


 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  roshand_gen2_msgs
  rospy
)


FIND_LIBRARY(YAML_CPP_LIBRARIES yaml-cpp)
if(NOT YAML_CPP_LIBRARIES)

  FIND_PACKAGE(yaml-cpp REQUIRED)
  INCLUDE_DIRECTORIES(${YAML_CPP_INCLUDE_DIRS})
endif(NOT YAML_CPP_LIBRARIES)

catkin_package()


include_directories(
 include ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(/usr/local/lib)
include_directories(/usr/local/include/yaml-cpp)


add_executable(roshand_gen2_bringup
               src/roshand_gen2_bringup.cpp
               src/roshand_gen2_driver.cpp
 )
add_dependencies(roshand_gen2_bringup ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(roshand_gen2_bringup yaml-cpp ${catkin_LIBRARIES} ${Boost_LIBRARIES})


add_executable(sensor_marker
               src/sensor_marker.cpp
 )
add_dependencies(sensor_marker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(sensor_marker  ${catkin_LIBRARIES} )
