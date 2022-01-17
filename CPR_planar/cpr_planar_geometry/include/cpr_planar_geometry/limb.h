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



#ifndef LIMB_H
#define LIMB_H

#include "cpr_planar_geometry/rod.h"
#include "cpr_planar_geometry/joint.h"



namespace cpr_planar_geometry {

class Limb
{
public:
  Limb()=default;


  Limb(std::shared_ptr<cpr_planar_geometry::AbstractJoint> base_joint,
       std::shared_ptr<cpr_planar_geometry::AbstractJoint> distal_plate_joint) : base_joint_(base_joint), distal_plate_joint_(distal_plate_joint) {}

  Limb(std::shared_ptr<cpr_planar_geometry::AbstractJoint> base_joint,
       std::shared_ptr<cpr_planar_geometry::AbstractJoint> distal_plate_joint,
       const Rod &rod_, const bool is_extendible=false) : base_joint_(base_joint), distal_plate_joint_(distal_plate_joint),
                                                          rod(rod_), is_extendible_(is_extendible), joint_value(0.0) {
       std::cout << "This is limb is " << (is_extendible_ ? "" : "not ") << "extendible" << std::endl;
      }
  /*!
   * \brief EvaluateBoundaryConditions numerically integrates the rod and then evaluates the constrains at the distal plate
   * \param distal_plate_transf is current poisiton of the distal plate
   * \param guess is the current guessed parameters vector relative to the limb
   * \return a residual vector containing the errors coming from the effect of the current parameter vector.
   *
   * In this function, first the guess vector is decomposed to obtain the values of interest. The joint value element of
   * the parameter vector is used to compute the rod initial frame given the base joint. At this point the state vector is
   * built with the states [p0, th0, n0, m0] and the rod is numerically integrated.
   *
   * After integration, the distal plate joint is used to check the assembly constrains. Specifically the position error,
   * angle error and the joint constrain are evaluated. For the latter, the joint constrain can be, taking a revolute joint
   * as example, that the moment the rod applies on the joint must be zero.
   *
   * After all the constrains are evaluated, they are lumped in a vector and returned
   */
  Eigen::VectorXd EvaluateBoundaryConditions(const Eigen::Affine2d &distal_plate_transf, const Eigen::Vector4d &guess);

  /*!
   * \brief GetRodVisual numerically integrates the rod with the given parameters
   * \param n0 initial forces
   * \param m0 initial moment
   * \return a RodCenterline element containing all the centerline points.
   *
   * This function uses the last updeted values of the joint obtained from EvaluateBoundaryConditions().
   * It sets up the state vector and numericlly integrate the rod using an observed to obtain the points
   * on the centerline.
   */
  cpr_planar_visual::RodCenterline GetRodVisual(const Eigen::Vector2d &n0,
                                           const double &m0);

  /*!
   * \brief GetAppliedWrench returns the wrench this limbs applies on the distal plate
   */
  inline Eigen::Vector3d GetAppliedWrench() const {return wrench_applied_to_distal_plate;}

  /*!
   * \brief GetJointValue returns the last joint value obtained from the shooting method
   */
  inline double GetJointValue() const {return joint_value;}

private:
  //  Each limb as one joint at the base and one joint at the distal plate
  std::shared_ptr<AbstractJoint> base_joint_;
  std::shared_ptr<AbstractJoint> distal_plate_joint_;

  //  Each limb as one rod
  Rod rod;
  const bool is_extendible_ { false };

  //  Current value of its base joint
  double joint_value;

  //  Instance of the local wrench balance
  Eigen::Vector3d wrench_applied_to_distal_plate { Eigen::Vector3d::Zero() };


};


} //  END namespace cpr_planar_geometry



#endif // LIMB_H
