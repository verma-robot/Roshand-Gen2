cmake_minimum_required(VERSION 3.0.2)
project(roshand_gen2_msgs)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)

 add_message_files(
   FILES
   Finger.msg
   Motor.msg
   Hand.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
   FILES
   SensorCalibrate.srv
#   Service2.srv
 )

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   HandCommand.action
#   Action2.action
 )


 generate_messages(
   DEPENDENCIES std_msgs actionlib_msgs
 )

catkin_package(
  CATKIN_DEPENDS actionlib actionlib_msgs roscpp rospy sensor_msgs std_msgs std_srvs
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

