cmake_minimum_required(VERSION 3.0.2)
project(cpr_planar_geometry)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)
#set(CMAKE_BUILD_TYPE Debug)

#find_package(gazebo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cpr_planar_visual
)

#   Find Boost for the odeint integrators
find_package(Boost REQUIRED COMPONENTS system)

#   Find Eigen for vectors, matrices and general operations
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


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

## Declare a C++ library
add_library(${PROJECT_NAME}     src/rod.cpp
                                src/joint.cpp
                                src/limb.cpp
                                )
target_include_directories(${PROJECT_NAME} PRIVATE include ${EIGEN3_INCLUDE_DIRS} )
target_link_libraries(${PROJECT_NAME}  ${catkin_LIBRARIES} )
