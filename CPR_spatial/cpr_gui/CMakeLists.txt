cmake_minimum_required(VERSION 3.0.2)
project(cpr_gui)
set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_BUILD_TYPE Debug)


# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
#set(CMAKE_AUTORCC ON)
#set(CMAKE_AUTOUIC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Widgets finds its own dependencies (QtGui and QtCore).
find_package(Qt5Widgets REQUIRED)

find_package(YamlCpp)


# The Qt5Widgets_INCLUDES also includes the include directories for
# dependencies QtCore and QtGui
include_directories(${Qt5Widgets_INCLUDES})

# We need add -DQT_WIDGETS_LIB when using QtWidgets in Qt 5.
add_definitions(${Qt5Widgets_DEFINITIONS})

# Executables fail to build with Qt 5 in the default configuration
# without -fPIE. We add that here.
set(CMAKE_CXX_FLAGS "${Qt5Widgets_EXECUTABLE_COMPILE_FLAGS}")
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++0x")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  cpr_messages
  urdf
)




catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_gui
  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
 include
 src
 ${catkin_INCLUDE_DIRS}
)

#   Add a library with the definition of the generic widgets for control
add_library(generic_controllers src/controllers/generic_panel.cpp
                                src/controllers/generic_controller.cpp)
target_link_libraries(generic_controllers ${Qt5Widgets_LIBRARIES})

#   Add a library with the implementation of varius panel for controls
add_library(control_panels  src/distal_plate_pose_panel/distal_plate_pose_panel.cpp
                            src/distal_plate_wrench_panel/distal_plate_wrench_panel.cpp
                            src/joints_control_panel/joints_control_panel.cpp
                            src/solver_options_panel/solver_options_panel.cpp)
target_link_libraries(control_panels    generic_controllers
                                        ${Qt5Widgets_LIBRARIES}
                                        ${catkin_LIBRARIES})
target_compile_definitions(control_panels PUBLIC ROS)

#   Add a library to handle the saving and load of the parameters vector
add_library(parameters_vector_manager src/parameters_vector_manager/parameters_vector_loader.cpp
                                      src/parameters_vector_manager/parameters_vector_saving.cpp)
target_link_libraries(parameters_vector_manager yaml-cpp ${Qt5Widgets_LIBRARIES})

#   Add a library to handle the dof selection for an underactuated robot
add_library(dof_selection_window    src/dof_selection_window/basic_motion.cpp
                                    src/dof_selection_window/dof_controller.cpp
                                    src/dof_selection_window/dof_selection_window.cpp)
target_link_libraries(dof_selection_window ${Qt5Widgets_LIBRARIES} ${catkin_LIBRARIES})
target_compile_definitions(dof_selection_window PUBLIC ROS)

#   Add a library with the mainwindow utilities
add_library(main_window_utils src/main_window/menubar.cpp)
target_link_libraries(main_window_utils ${Qt5Widgets_LIBRARIES} ${catkin_LIBRARIES})
target_compile_definitions(main_window_utils PUBLIC ROS)

add_library(main_window src/main_window/main_window.cpp)
target_link_libraries(main_window   main_window_utils
                                    generic_controllers
                                    control_panels
                                    parameters_vector_manager
                                    dof_selection_window
                                    yaml-cpp
                                    ${Qt5Widgets_LIBRARIES}
                                    ${catkin_LIBRARIES})
target_compile_definitions(main_window PUBLIC ROS)

#   Add executable for using the GUI
add_executable(${PROJECT_NAME}_node src/cpr_gui.cpp)
target_link_libraries(${PROJECT_NAME}_node main_window ${catkin_LIBRARIES} ${Qt5Widgets_LIBRARIES})
