/*!
MIT License

Copyright (c) 2022 Gotelli Andrea

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/




#ifndef CPR_PLANAR_PHYSICS_H
#define CPR_PLANAR_PHYSICS_H

#include <map>
#include <random>
#include <iostream>

#include "cpr_planar_geometry/limb.h"

#include "cpr_planar_visual/shape_publisher.h"

#include "cpr_planar_messages/rod_shape.h"


#include <ros/ros.h>
#include <control_msgs/JointControllerState.h>


#include <ceres/ceres.h>  //  Include library with LM solver


namespace physics_core {

enum ProblemType {
  IGM,
  DGM
};

struct Wrench {
  Wrench()=default;
  double fx { 0.0 };
  double fy { 0.0 };
  double mz { 0.0 };
};


class RobotPhysics
{
public:

  RobotPhysics()=default;


  /*!
   * \brief operator () Is the shooting function which will be evaluted by the LM algorithm
   * \param guess Is the current guess vector containing the parameters for the robot
   * \param residuals vector containing the error coming from the evaluation of the current guess
   * \return true, no possibility of failure
   */
  bool operator()(double const* const* guess, double* residuals) const;

  /*!
   * \brief Initialize initializes the robot model, with a visual output.
   * \param distal_plate_pose_ is the current position of the distal plate.
   *
   * This function initializes the robot, solving the assembly and displaing both the
   * rod centerlines and the solver output.
   */
  void InitializeRobotModel(const Eigen::Affine2d& distal_plate_pose,
                            const std::vector<cpr_planar_geometry::Limb> &robot_limbs,
                            const std::vector<double> &initial_guess,
                            std::vector<double> &joint_values);


  void SolveIGM(const Eigen::Affine2d &_distal_plate_pose, std::vector<double> &_joint_values, const Wrench &wrench=Wrench());

  void SolveDGM(Eigen::Affine2d &_distal_plate_pose, const std::vector<double> &_joint_values, const Wrench &wrench=Wrench());

  [[nodiscard]] bool SetGuessVector(std::vector<double> given_guess,
                                      Eigen::Affine2d &distal_plate_pose,
                                      std::vector<float> &joint_values);

  template<class Container>
  void GetCurrentGuessVector(Container &guess_copy) const
  {
    guess_copy.resize(guess_size_);

    for (unsigned int i=0;i<guess_size_;i++)
      guess_copy[i] = guess_[i];

  }
private:

  enum VECTOR_DIMENSIONS {
    LIMB_GUESS = 4,
    DISTAL_PLATE_GUESS = 3,
    LIMB_RESIDUAL = 4,
    EQUILIBRIUM_SECTION = 3
  };

  void InitilizeVectorsDimensions(const unsigned int _number_of_limbs)
  {
    number_of_limbs_ = _number_of_limbs;
    guess_size_ = VECTOR_DIMENSIONS::LIMB_GUESS*number_of_limbs_ + VECTOR_DIMENSIONS::DISTAL_PLATE_GUESS;
    residual_vector_size_ = VECTOR_DIMENSIONS::LIMB_RESIDUAL*number_of_limbs_ + VECTOR_DIMENSIONS::EQUILIBRIUM_SECTION + VECTOR_DIMENSIONS::DISTAL_PLATE_GUESS;
  }

  unsigned int number_of_limbs_;
  unsigned int guess_size_;
  unsigned int residual_vector_size_;

  //  Local instance of the vector of guessed parameters
  mutable std::vector<double> guess_;

  //  All the robot limbs
  mutable std::vector<cpr_planar_geometry::Limb> robot_limbs_;

  //  Values of the limbs joints
  mutable std::vector<double> joints_value_;

  //  A local instance of the distal plate position wrt the world frame
  mutable Eigen::Affine2d distal_plate_pose_ { Eigen::Affine2d::Identity() };

  //  Visual publisher using ZeroMQ and relative message;
  cpr_planar_visual::ShapePublisher shapes_publisher_;
  cpr_planar_visual::ShapeMsg shapes_message_;

  //  Define the problem
  ceres::Problem problem_;

  //  Solver Options
  ceres::Solver::Options options_;

  //  Solver verbosity
  ceres::Solver::Summary summary_;

  //  Costfunction which will be minimized
  ceres::DynamicNumericDiffCostFunction<RobotPhysics>* cost_function_;


  ProblemType problem_type_;

  Wrench wrench_;

  /*!
   * \brief PublishRodsVisual this function publishes the visualization message for the visual plugin
   *
   * This function first numerically integrates each rod from the solution obtained from the shooting method.
   * Then it initialized the visualization message and publish it with the visual publisher.
   */
  void PublishRodsVisual();

  void LogGuessWithLabel(const std::string &label) const;

  [[nodiscard]] bool TryToSolveAssembly();

  void GuessNullInit() const;

  void CheckForSolverIssue() const;


};





} //  END namespace physics_core


#endif // CPR_PLANAR_PHYSICS_H
