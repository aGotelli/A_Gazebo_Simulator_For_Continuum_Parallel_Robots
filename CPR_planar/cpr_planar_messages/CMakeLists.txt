cmake_minimum_required(VERSION 3.0.2)
project(cpr_planar_messages)


find_package(catkin REQUIRED COMPONENTS
    message_generation
    geometry_msgs
)


################################################
## Declare ROS messages, services and actions ##
################################################


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  rod_shape.msg
  JointValue.msg
  JointsControl.msg
  Wrench2D.msg
  DGMParameters.msg
  IGMParameters.msg
)

add_service_files(
	FILES
	GuessVectorHandling.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
)



###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_messages
  CATKIN_DEPENDS geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

