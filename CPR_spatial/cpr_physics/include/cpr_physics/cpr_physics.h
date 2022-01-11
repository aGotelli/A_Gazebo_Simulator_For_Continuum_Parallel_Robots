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




#ifndef CPR_PHYSICS_H
#define CPR_PHYSICS_H

#include <map>
#include <random>
#include <fstream>
#include <iostream>

#include "cpr_geometry/limb.h"

#include "cpr_visual/shape_publisher.h"

#include <yaml-cpp/yaml.h>

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
  Eigen::Vector3d force { Eigen::Vector3d::Zero() };
  Eigen::Vector3d torque { Eigen::Vector3d::Zero() };
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
  void InitializeRobotModel(Eigen::Affine3d &distal_plate_pose,
                            const std::vector<cpr_geometry::Limb> &robot_limbs,
                            const std::vector<double> &initial_guess,
                            std::vector<float> &joint_values,
                            const Eigen::Matrix<double, 6, 6> &underactuation_correction=Eigen::Matrix<double, 6, 6>::Identity(),
                            const Wrench &initial_wrench=Wrench());


  void SolveIGM(const Eigen::Affine3d &_distal_plate_pose, std::vector<float> &_joint_values, const Wrench &wrench=Wrench());

  void SolveDGM(Eigen::Affine3d &_distal_plate_pose, const std::vector<float> &_joint_values, const Wrench &wrench=Wrench());


  [[nodiscard]] bool SetGuessVector(std::vector<double> given_guess,
                                    Eigen::Affine3d &distal_plate_pose,
                                    std::vector<float> &joint_values);

  template<class Container>
  void GetCurrentGuessVector(Container &guess_copy) const
  {
    guess_copy.resize(guess_size_);

    for (unsigned int i=0;i<guess_size_;i++)
      guess_copy[i] = guess_[i];

  }

  void RemoveLines()
  {
    cpr_visual::ShapeMsg default_message;
    default_message.shapes_to_define = cpr_visual::VISUAL_PROPERTIES::MAX_NUMBER_OF_RODS;

    shapes_publisher_.publish( default_message );
  }


private:

  enum VECTOR_DIMENSIONS {
    LIMB_GUESS = 7,         //  [q, nx, ny, nz, mx, my, mz  ]
    DISTAL_PLATE_GUESS = 6, //  [x, y, z, roll, pitch, yaw  ]
    LIMB_RESIDUAL = 9,      //  [ex, ey, ez, eψ, eφ, eθ, jc1, jc2, jc3  ]
    EQUILIBRIUM_SECTION = 6 //  [fx, fy, fz, mx, my, mz ]
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
  mutable std::vector<cpr_geometry::Limb> robot_limbs_;

  //  Values of the limbs joints
  mutable std::vector<float> joints_value_;

  //  A local instance of the distal plate position wrt the world frame
  mutable Eigen::Affine3d distal_plate_pose_ { Eigen::Affine3d::Identity() };

  //  Visual publisher using ZeroMQ and relative message;
  cpr_visual::ShapePublisher shapes_publisher_;
  cpr_visual::ShapeMsg shapes_message_;

  //  Define the problem
  ceres::Problem problem_;
  ceres::ResidualBlockId problem_id;

  //  Solver Options
  ceres::Solver::Options options_;

  //  Solver verbosity
  ceres::Solver::Summary summary_;

  //  Costfunction which will be minimized
  ceres::DynamicNumericDiffCostFunction<RobotPhysics>* cost_function_;


  ProblemType problem_type_;

  Wrench wrench_;
  Wrench initial_wrench_;

  /*!
   * \brief PublishRodsVisual this function publishes the visualization message for the visual plugin
   *
   * This function first numerically integrates each rod from the solution obtained from the shooting method.
   * Then it initialized the visualization message and publish it with the visual publisher.
   */
  void PublishRodsVisual();

  void LogGuessWithLabel(const std::string &label) const;

  Eigen::Affine3d DistalPlatePoseFromParametersVector() const;

  [[nodiscard]] bool TryToSolveAssembly();

  void GuessNullInit() const;
  void GuessInitDistalPlateSection() const;

  void CheckForSolverIssue() const;


  const Eigen::Affine3d ToAffine3D(const Eigen::VectorXd &dp_guess_section) const;

  Eigen::Matrix<double, 6, 6> underactuation_correction_ { Eigen::Matrix<double, 6, 6>::Identity() };


};





} //  END namespace physics_core


#endif // CPR_PHYSICS_H
