# A_Gazebo_Simulator_For_Continuum_Parallel_Robots

This repository contains the simulator for Continuum Parallel Robots (CPR)s.
There are two folders containing the two versions of the simulator: one for planar and one for spatial CPRs.

The simulators layout and usage are described in the related [video](https://youtu.be/6k5aZPOQjQ8).

## Dependencies

Both simulators relies on the Ceres solver, the Qt libraries, ROS and Gazebo.
The non linear solver Ceres can be downloaded following the instruction on the [Ceres website](http://ceres-solver.org/installation.html), and the [Qt libraries](https://www.qt.io/download) as well.

For what concers ROS and Gazebo, they can be installed following the [online instruction](http://wiki.ros.org/noetic/Installation) selecting the "Desktop full install". If Gazebo is not directly installed, then it can be installed with the [online instructions](http://gazebosim.org/tutorials?tut=install_ubuntu).