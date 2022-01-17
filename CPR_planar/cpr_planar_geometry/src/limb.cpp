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



#include "cpr_planar_geometry/limb.h"


using namespace Eigen;

namespace cpr_planar_geometry {



Eigen::VectorXd Limb::EvaluateBoundaryConditions(const Affine2d &distal_plate_transf, const Eigen::Vector4d& guess)
{
  //  Decompose the guess vector
  if(is_extendible_) {
    rod.SetLenght(guess[0]);
  } else joint_value = guess[0];

  const Eigen::Vector2d n0(guess[1], guess[2]);
  const double m0 = guess[3];

  //  Compute rod initial position and orientation
  const Eigen::Affine2d rod_init_frame = base_joint_->GetRodInitialFrame(joint_value);
  const Eigen::Vector2d p0 = rod_init_frame.translation();
  const double th0 = Eigen::Rotation2D<double>( rod_init_frame.linear() ).angle();

  //  Compose the state vector
  state_type y;
  y << p0, th0, n0, m0;
  //   0 1  2   3 4  5

  /*  This is faster, less copies, but still semantic as before?
  y <<  rod_init_frame.translation(),
        Eigen::Rotation2D<double>( rod_init_frame.linear() ).angle(),
        guess.segment(3, 3);
  */

  //  numerically integrate the rod to obtain the state at the tip
  rod.Integrate(y);

  //  Obtain the Transformation of the last rod cross-section
  Eigen::Affine2d rod_tip_transform;
  rod_tip_transform.translation() = Eigen::Vector2d(y.x(), y.y());
  rod_tip_transform.linear() = Eigen::Rotation2D<double>( y[2] ).toRotationMatrix();

  const Eigen::Vector2d n_L(y[3], y[4]);
  const double m_L = y[5];

  //  The wrench at the rod tip in world coordinates
  const Eigen::Vector3d rod_tip_wrench(-n_L.x(), -n_L.y(), -m_L);

  //  Store the current wrench this limbs applies on the distal plate
  wrench_applied_to_distal_plate = distal_plate_joint_->ComputeAppliedWrench(distal_plate_transf, rod_tip_transform, rod_tip_wrench);

  //  Evaluate geometrical quantities
  const Vector2d position_error = distal_plate_joint_->evaluatePositionError(distal_plate_transf, rod_tip_transform);
  const auto angle_error = distal_plate_joint_->evaluateAngleError(distal_plate_transf, rod_tip_transform);

  //  Evaluate joint constraint
  const double joint_constraint = distal_plate_joint_->GetWrenchConstrain(rod_tip_wrench, distal_plate_transf);

  //  Define the result of the integration
  VectorXd result(4);
  result << position_error, angle_error, joint_constraint;

  /*  Sure, faster, but worth it? still semantinc as before?
  result << distal_plate_joint->evaluatePositionError(distal_plate_transf, rod_tip_transform),
            distal_plate_joint->evaluateAngleError(distal_plate_transf, rod_tip_transform),
            distal_plate_joint->GetWrenchConstrain(rod_tip_wrench, distal_plate_transf),
  */

  return result;
}



cpr_planar_visual::RodCenterline AdaptVectorsLenght(std::valarray<cpr_planar_visual::Point> rod_centerline)
{
  //  Slice the vector ensuring the required lenght
  std::valarray<cpr_planar_visual::Point> resized_rod_centerline = rod_centerline[std::slice(0,
                                      cpr_planar_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE,
                                      static_cast<unsigned int>(rod_centerline.size()/
                                                                cpr_planar_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE))] ;
  //  Ensure last term is equal in both vectors
  resized_rod_centerline[resized_rod_centerline.size()-1] = rod_centerline[rod_centerline.size()-1];

  cpr_planar_visual::RodCenterline rod_visual;

  std::copy_n( begin(resized_rod_centerline),
               cpr_planar_visual::VISUAL_PROPERTIES::POINTS_PER_CENTERLINE,
               rod_visual.points.begin() );

  return rod_visual;
}


cpr_planar_visual::RodCenterline Limb::GetRodVisual(const Vector2d &n0, const double &m0)
{
  //  Obtain rod first frame
  const auto rod_init_frame = base_joint_->GetRodInitialFrame(joint_value);

  //  Obtain needed quantities
  const Eigen::Vector2d p0 = rod_init_frame.translation();
  const double th0 = Eigen::Rotation2D<double>( rod_init_frame.linear() ).angle();


  //  Define the initial state vector of the rod
  state_type y;
  y << p0, th0, n0, m0;

  //  Obtain the visualization of the rod with the given IVP
  cpr_planar_visual::RodCenterline rod_visual = rod.IntegrateWithObserver(y);


/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *   TEMPORARY SOLUTION, NEED TO RETRIEVE CURRENT ATTACH POINT Z COORDINATE
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
  for(auto& point : rod_visual.points)
    point.z = 0.05;

  //  Integrate with visual outup
  return rod_visual;
}




} //  END namespace cpr_planar_geometry

