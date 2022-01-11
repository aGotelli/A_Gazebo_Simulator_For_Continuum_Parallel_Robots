cmake_minimum_required(VERSION 3.0.2)
project(cpr_physics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

find_package(gazebo)

#find_package(log2plot)

find_package(Ceres REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
)

find_package(YamlCpp)
include_directories(${YAML_CPP_INCLUDE_DIRS})

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_msgs
  cpr_visual
  cpr_geometry
  cpr_messages
  dynamic_reconfigure
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)




#   Add a library containing the cpr physics
add_library(${PROJECT_NAME} src/robot_physics)
target_include_directories(${PROJECT_NAME} PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${CERES_LIBRARIES} yaml-cpp)

#   Add a library containing the physics plugin
add_library(${PROJECT_NAME}_plugin src/physics_plugin.cpp)
target_include_directories(${PROJECT_NAME}_plugin PRIVATE include {EIGEN3_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME} ${GAZEBO_LIBRARY_DIRS})

#   Add an executable for implementing the closed loop control
add_executable(${PROJECT_NAME}_controller src/controller.cpp)
target_link_libraries(${PROJECT_NAME}_controller ${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
#target_link_libraries(${PROJECT_NAME}_controller log2plot)
