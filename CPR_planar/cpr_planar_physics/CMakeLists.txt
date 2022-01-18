cmake_minimum_required(VERSION 3.0.2)
project(cpr_planar_physics)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

find_package(gazebo)

find_package(Ceres REQUIRED)

find_package(catkin REQUIRED COMPONENTS
roscpp
)

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
  cpr_planar_visual
  cpr_planar_geometry
  cpr_planar_messages
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



#   Add a library containing the robot physics
add_library(${PROJECT_NAME} src/cpr_planar_physics.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${EIGEN3_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} ${CERES_LIBRARIES})

#   Add a library containing the physics plugin
add_library(${PROJECT_NAME}_plugin src/physics_plugin.cpp)
target_include_directories(${PROJECT_NAME}_plugin PRIVATE include {EIGEN3_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_plugin ${PROJECT_NAME} ${GAZEBO_LIBRARY_DIRS})
