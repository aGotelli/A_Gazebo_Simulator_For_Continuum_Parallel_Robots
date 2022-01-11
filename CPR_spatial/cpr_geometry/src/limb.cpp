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

#include "cpr_geometry/limb.h"


using namespace Eigen;

namespace cpr_geometry {





Eigen::VectorXd Limb::EvaluateBoundaryConditions(const Affine3d &distal_plate_transf, const Eigen::VectorXd& guess)
{
/* The guess vector is composed as follows
 * [  q, nx, ny, nz, mx, my, mz  ]
 * [  0,  1,  2,  3,  4,  5,  6  ]
 *
 * And from this we unpack the guess vector
*/
  //  Check if the limb is an extensible one (then q referes to the rod lenght)
  if(is_extensible_) {
    rod.SetLenght(guess[0]);
  } else joint_value = guess[0];

  const Vector3d n0(guess[1], guess[2], guess[3]);
  const Vector3d m0(guess[4], guess[5], guess[6]);

  //  Compute rod initial position and orientation
  const Affine3d rod_init_frame = base_joint_->GetRodInitialFrame(joint_value);
  const Vector3d p0 = rod_init_frame.translation();
  Matrix3d R0 = rod_init_frame.linear();

  //  Compose the state vector
  state_type y;
  y << p0, Map<VectorXd>(R0.data(), R0.size()), n0, m0;

  //  numerically integrate the rod to obtain the state at the tip
  rod.Integrate(y);
  /* The state vector is composed as follows
   * [  px, py, pz, r11, ..., r33, nx, ny, nz, mx, my, mz  ]
   * [   0,  1,  2,   3, ...,  11, 12, 13, 14, 15, 16, 17  ]
   *
   * And from this we unpack the state vector
  */

  //  Obtain the Transformation of the last rod cross-section (the rod tip)
  Eigen::Affine3d rod_tip_transform;
  rod_tip_transform.translation() = Eigen::Vector3d(y[0], y[1], y[2]);
  rod_tip_transform.linear() = Matrix3d(y.segment(3,9).data());

  //  Compute the internal forces and moment at the rod tip
  Vector3d n_L(y[12], y[13], y[14]);
  Vector3d m_L(y[15], y[16], y[17]);

  //  The wrench at the rod tip in world coordinates
  Eigen::VectorXd rod_tip_wrench(6);
  rod_tip_wrench << -n_L, -m_L;

  //  Store the current wrench this limbs applies on the distal plate
  wrench_applied_to_distal_plate = distal_plate_joint_->ComputeAppliedWrench(distal_plate_transf,
                                                                             rod_tip_wrench,
                                                                             rod_tip_transform);

  //  Evaluate geometrical quantities
  const Vector3d position_error = distal_plate_joint_->evaluatePositionError(distal_plate_transf, rod_tip_transform);
  const Vector3d orientation_error = distal_plate_joint_->evaluateOrientationError(distal_plate_transf, rod_tip_transform);

  //  Evaluate joint constraint
  const Vector3d joint_constraint = distal_plate_joint_->GetJointWrenchConstrain(rod_tip_wrench, distal_plate_transf);

  //  Define the result of the integration
  VectorXd result(9);
  result << position_error, orientation_error, joint_constraint;


//  result << distal_plate_joint_->evaluatePositionError(distal_plate_transf, rod_tip_transform),
//            distal_plate_joint_->evaluateOrientationError(distal_plate_transf, rod_tip_transform),
//            distal_plate_joint_->GetWrenchConstrain(rod_tip_wrench, distal_plate_transf);


  return result;
}


cpr_visual::RodCenterline Limb::GetRodVisual(const Vector3d &n0, const Vector3d &m0)
{
  //  Obtain rod first frame
  const auto rod_init_frame = base_joint_->GetRodInitialFrame(joint_value);

  //  Obtain needed quantities
  const Eigen::Vector3d p0 = rod_init_frame.translation();
  Matrix3d R0 = rod_init_frame.linear();


  //  Define the initial state vector of the rod
  state_type y;
  y << p0, Map<VectorXd>(R0.data(), R0.size()), n0, m0;

  //  Obtain the visualization of the rod with the given IVP
  cpr_visual::RodCenterline rod_visual = rod.IntegrateWithObserver(y);


  //  Integrate with visual outup
  return rod_visual;
}




} //  END namespace cpr_geometry

