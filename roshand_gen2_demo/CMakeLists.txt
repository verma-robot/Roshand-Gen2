cmake_minimum_required(VERSION 3.0.2)
project(roshand_gen2_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  roshand_gen2_msgs
)

catkin_package()

include_directories(  ${catkin_INCLUDE_DIRS})

add_executable(demo  src/demo.cpp )
add_dependencies(demo ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) 

target_link_libraries(demo  ${catkin_LIBRARIES} )
