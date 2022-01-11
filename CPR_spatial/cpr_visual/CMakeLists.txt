cmake_minimum_required(VERSION 3.0.2)
project(cpr_visual)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)


find_package(gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_path ( ZMQ_INCLUDE_DIR zmq.h )
find_library ( ZMQ_LIBRARY NAMES zmq )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  #CATKIN_DEPENDS roscpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#   Library with the listeners and publishers of the shapes message
add_library(${PROJECT_NAME}    src/shape_listener.cpp
                               src/shape_publisher.cpp )
target_include_directories(${PROJECT_NAME} PRIVATE  include ${ZMQ_INCLUDE_DIR} )
target_link_libraries(${PROJECT_NAME}   ${catkin_LIBRARIES} ${ZMQ_LIBRARY} )

#   Library with the plugin to be included in Gazebo model
add_library(${PROJECT_NAME}_plugin src/visual_plugin.cpp)
target_include_directories(${PROJECT_NAME}_plugin PRIVATE   include ${GAZEBO_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME}_plugin    ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARY_DIRS} )
