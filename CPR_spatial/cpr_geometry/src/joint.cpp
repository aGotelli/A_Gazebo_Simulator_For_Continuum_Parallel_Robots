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




#include "cpr_geometry/joint.h"

using namespace Eigen;

namespace cpr_geometry{




FixedJoint::FixedJoint(const Affine3d& origin_, const bool verbose) : origin(origin_)
{
  Adg_rj.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
  Adg_rj.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Adg_rj.block<3,3>(3, 0) = Eigen::Matrix3d::Zero();
  Adg_rj.block<3,3>(3, 3) = Eigen::Matrix3d::Identity();

  Adg_jd.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
  Adg_jd.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Adg_jd.block<3,3>(3, 0) = Eigen::Matrix3d::Zero();
  Adg_jd.block<3,3>(3, 3) = Eigen::Matrix3d::Identity();

  if(verbose) {
    std::cout << "Initialized joint with origin :" << std::endl;
    std::cout << origin.matrix() << std::endl;
  }
}


const Affine3d FixedJoint::GetRodInitialFrame(const double&) const
{
  return origin;
}


const Vector3d FixedJoint::evaluatePositionError(const Affine3d& distal_plate_transf,
                                                 const Affine3d& rod_tip_transf) const
{
  return Vector3d( rod_tip_transf.translation() ) -
         Vector3d( (distal_plate_transf*origin ).translation() ) ;
}



const Vector3d FixedJoint::evaluateOrientationError(const Affine3d& distal_plate_transf,
                                                    const Affine3d& rod_tip_transf) const
{
  Matrix3d wRj = (distal_plate_transf*origin).linear();

  Matrix3d wRrap = (rod_tip_transf).linear();

  Vector3d e_rot = inv_hat(wRj.transpose()*wRrap - wRj*wRrap.transpose());

  return e_rot;
}



const Eigen::VectorXd FixedJoint::ComputeAppliedWrench(const Affine3d &distal_plate_transf,
                                                       const VectorXd &rod_tip_wrench,
                                                       const Affine3d &rod_tip_transf) const
{
  //  Vector from rot tip to joint origin
  const Vector3d p_rj = (distal_plate_transf*origin).translation() - rod_tip_transf.translation();


  //  Update Adjoint tranformation
  Adg_rj.block<3,3>(3, 0) = -hat(p_rj);

  //  Shifting law for the wrench (compute its values at the joint origin)
  const VectorXd wrench_jo = Adg_rj*rod_tip_wrench;

  //  Vector from joint origin to distal plate orgin (wrt reference frame)
  const Vector3d p_jdp = -distal_plate_transf.linear()*origin.translation();

  //  update Adjoint tranformation
  Adg_jd.block<3,3>(3, 0) = -hat(p_jdp);


  return Adg_jd*wrench_jo;
}



const Affine3d RevoluteJoint::GetRodInitialFrame(const double& joint_value) const
{
  Matrix3d R;
  R = AngleAxisd(joint_value, rotation_axis);

  Affine3d T = Affine3d::Identity();
  T.linear() = R;

  return origin * T * attach_point;
}

const Eigen::Vector3d RevoluteJoint::evaluatePositionError(const Affine3d& distal_plate_transf,
                                                           const Affine3d& rod_tip_transf) const
{
  return (distal_plate_transf*origin).translation() -
          (rod_tip_transf*attach_point.inverse()).translation();
}




const Vector3d RevoluteJoint::evaluateOrientationError(const Eigen::Affine3d &distal_plate_transf,
                                                       const Eigen::Affine3d &rod_tip_transf) const
{
  //  Get the orieintation of the joint frame
  Matrix3d wRj = (distal_plate_transf*origin).linear();

  //  Compute what would be the orientation of the joint origin reached from the
  //  inverse of the rigid body transform of the attach point
  Matrix3d wRrap = (rod_tip_transf*attach_point.inverse()).linear();

  //  Compute the error betwee these two orientations
  Vector3d wRe_tilde = inv_hat(wRj.transpose()*wRrap - wRj*wRrap.transpose());

  //  Project this error in the joint frame
  Vector3d jRe_tilde = wRj.transpose()*wRe_tilde;

  Matrix3d I_tilde = Matrix3d::Identity() - Scaling(rotation_axis).toDenseMatrix();

  //  Get the orientation error which accounts for the joint dof
  Vector3d jRe = I_tilde*jRe_tilde;

  //  Reproject the error in the world coordinates
  Vector3d wRe = wRj*jRe;

  return wRe;
}


const Eigen::VectorXd RevoluteJoint::ComputeAppliedWrench(const Affine3d &distal_plate_transf,
                                                          const Eigen::VectorXd &rod_tip_wrench,
                                                          const Eigen::Affine3d &rod_tip_transf) const
{
  Affine3d wTj = distal_plate_transf*origin;
  Matrix3d wRj = wTj.linear();

  //  Vector from rot tip to joint origin (in world coordinates)
  const Vector3d wp_rj = wTj.translation() - rod_tip_transf.translation();


  //  Adjoint tranformation
  Eigen::Matrix<double, 6, 6> Adg_rj;
  Adg_rj.block<3,3>(0, 0) = wRj.transpose();
  Adg_rj.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Adg_rj.block<3,3>(3, 0) = -wRj.transpose()*hat(wp_rj);
  Adg_rj.block<3,3>(3, 3) =  wRj.transpose();

  //  Shifting law for the wrench (compute its values at the joint origin in joint coordinates)
  const VectorXd jwrench_j_tilde = Adg_rj*rod_tip_wrench;

  //  Correct the wrench accounting for joint DOF
  const VectorXd jwrench_j = J_dof*jwrench_j_tilde;

  //  Account for this error as joint constraint (in relative coordinates)
  jjoint_cnstr = J_cnstr*jwrench_j_tilde;




  //  Vector from joint to distal plate in joint coordinates
  //  As we have the transform from distal plate to joint origin, computing
  //  the inverse we obtain the transform from the joint to the distal plate origin
  //  And then I extract the translation part.
  Vector3d jp_jdp = origin.inverse().translation();

  //  Adjoint tranformation
  Eigen::Matrix<double, 6, 6> Adg_jdp;
  Adg_jdp.block<3,3>(0, 0) = wRj;
  Adg_jdp.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Adg_jdp.block<3,3>(3, 0) = -wRj*hat(jp_jdp);
  Adg_jdp.block<3,3>(3, 3) =  wRj;

  //  Again shifting law computing wrench at the distal plate
  VectorXd wwrench_dp = Adg_jdp*jwrench_j;

  return wwrench_dp;

}

const Vector3d RevoluteJoint::GetJointWrenchConstrain(const Eigen::VectorXd &,
                                                 const Eigen::Affine3d &distal_plate_transf) const
{
  //  Get the Orientation of the joint frame
  Matrix3d wRj = (distal_plate_transf*origin).linear();

  //  Project the joint contraint in world coordinates
  Vector3d wjoint_cnstr = wRj*jjoint_cnstr;

  return wjoint_cnstr;
//  return Vector3d::Zero();

}




const Eigen::Vector3d SphericalJoint::evaluatePositionError(const Eigen::Affine3d &distal_plate_transf,
                                                            const Eigen::Affine3d &rod_tip_transf) const
{
  return (distal_plate_transf*origin).translation() -
          (rod_tip_transf*attach_point.inverse()).translation();
}


const Eigen::VectorXd SphericalJoint::ComputeAppliedWrench(const Affine3d &distal_plate_transf,
                                                           const Eigen::VectorXd &rod_tip_wrench,
                                                           const Eigen::Affine3d &rod_tip_transf) const
{
  //  Vector from rot tip to joint origin
  const Vector3d p_rj = (distal_plate_transf*origin).translation() - rod_tip_transf.translation();


  //  Update Adjoint transformation
  Adg_rj.block<3,3>(3, 0) = -hat(p_rj);

  //  Shifting law for the wrench (compute its values at the joint origin)
  const VectorXd wwrench_j = Adg_rj*rod_tip_wrench;

  //  Correct the wrench accounting for the null moment
  const VectorXd joint_wrench = J_dof*wwrench_j;
  jjoint_cnstr = J_cnstr*wwrench_j;

  //  Vector from joint origin to distal plate orgin (wrt reference frame)
  const Vector3d wp_jdp = -distal_plate_transf.linear()*origin.inverse().translation();

  //  Update Adjoint tranformation
  Adg_jd.block<3,3>(3, 0) = -hat(wp_jdp);


  return Adg_jd*wwrench_j;

}


const Eigen::Vector3d SphericalJoint::GetJointWrenchConstrain(const Eigen::VectorXd& rod_tip_wrench, const Eigen::Affine3d &distal_plate_transf) const
{
//  Vector3d r = attach_point.inverse().translation();

//  Eigen::Vector3d f_r = rod_tip_wrench.segment(0,3);
//  Eigen::Vector3d m_r = rod_tip_wrench.segment(3,3);

//  Vector3d moment_at_joint =  r.cross(f_r) + m_r;

//  return moment_at_joint;
  //  Get the Orientation of the joint frame
  Matrix3d wRj = (distal_plate_transf*origin).linear();

  //  Project the joint contraint in world coordinates
  Vector3d wjoint_cnstr = wRj*jjoint_cnstr;

  return wjoint_cnstr;

}




PrismaticJoint::PrismaticJoint(const Affine3d origin_, const Affine3d attach_point_, const bool verbose) :
  origin(origin_),
  attach_point(attach_point_){

  Adg_rj.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
  Adg_rj.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Adg_rj.block<3,3>(3, 0) = Eigen::Matrix3d::Zero();
  Adg_rj.block<3,3>(3, 3) = Eigen::Matrix3d::Identity();

  Adg_jd.block<3,3>(0, 0) = Eigen::Matrix3d::Identity();
  Adg_jd.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Adg_jd.block<3,3>(3, 0) = Eigen::Matrix3d::Zero();
  Adg_jd.block<3,3>(3, 3) = Eigen::Matrix3d::Identity();

  J_dof.block<3,3>(0,0) = Eigen::Matrix3d::Identity() - Eigen::Scaling(this->translation_axis).toDenseMatrix();
  J_dof.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
  J_dof.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
  J_dof.block<3,3>(3,3) = Eigen::Matrix3d::Identity();

  if(verbose){
    std::cout << "Intialized the prismatic joint." << std::endl;
    std::cout << "The attach point : " << attach_point.translation().transpose() << std::endl;
    std::cout << "The joint origin : " << std::endl;
    std::cout << origin.matrix() << std::endl;
    std::cout << "The attach point in local frame : " << std::endl;
    std::cout << attach_point.matrix() << std::endl;
    std::cout << "And dof correction matrix : " << std::endl;
    std::cout << J_dof << std::endl;
    std::cout << std::endl;
  }
}

const Affine3d PrismaticJoint::GetRodInitialFrame(const double& joint_value) const
{
  //  Get the joint translation
  Vector3d t = joint_value*translation_axis;

  //  Get the rigid transformation
  Affine3d T_joint_translation = Affine3d::Identity();
  T_joint_translation.translation() = t;

  //  Get the attach point 3D pose in world coordinates
  Affine3d wT_ap = origin*T_joint_translation*attach_point;

  return wT_ap;
}


const Vector3d PrismaticJoint::evaluatePositionError(const Affine3d &distal_plate_transf,
                                                     const Affine3d &rod_tip_transf) const
{
  //  Get the absolute position of the joint
  Vector3d wp_j = (distal_plate_transf*origin).translation();
  //  Orientation of the joint frame
  Matrix3d wRj = (distal_plate_transf*origin).linear();

  //  Get the absolute position of the reached point (rod and attach point)
  Vector3d wp_rap = (rod_tip_transf*attach_point.inverse()).translation();

  //  Compute the error vector
  Vector3d we_pos_tilde = wp_j - wp_rap;

  //  Project error in joint frame
  Vector3d je_pos_tilde = wRj.transpose()*we_pos_tilde;

  //  Matrix accounting for the DOF
  Matrix3d I_tilde = Matrix3d::Identity() - Scaling(translation_axis).toDenseMatrix();

  //  Account for the deegree of freedom
  Vector3d je_pos = I_tilde*je_pos_tilde;

  //  Reproject in absolute coordinates
  Vector3d we_pos = wRj*je_pos;

  return we_pos;

}


const Eigen::Vector3d PrismaticJoint::evaluateOrientationError(const Affine3d &distal_plate_transf,
                                                               const Affine3d &rod_tip_transf) const
{
  Matrix3d wRj = (distal_plate_transf*origin).linear();

  Matrix3d wRrap = (rod_tip_transf*attach_point.inverse()).linear();

  Vector3d we_ori = inv_hat(wRj.transpose()*wRrap - wRj*wRrap.transpose());

  return we_ori;
}

const Eigen::VectorXd PrismaticJoint::ComputeAppliedWrench(const Eigen::Affine3d &distal_plate_transf,
                                                           const Eigen::VectorXd &rod_tip_wrench,
                                                           const Eigen::Affine3d &rod_tip_transf) const
{
  Affine3d wTj = distal_plate_transf*origin;
  Matrix3d wRj = wTj.linear();

  //  Vector from rot tip to joint origin (in world coordinates)
  const Vector3d p_rj = wTj.translation() - rod_tip_transf.translation();

  //  Update Adjoint tranformation
  Adg_rj.block<3,3>(0, 0) = wRj.transpose();
  Adg_rj.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Adg_rj.block<3,3>(3, 0) = -wRj.transpose()*hat(p_rj);
  Adg_rj.block<3,3>(3, 3) =  wRj.transpose();

  //  Shifting law for the wrench (compute its values at the joint origin in joint coordinates)
  const VectorXd jwrench_j_tilde = Adg_rj*rod_tip_wrench;

  //  Correct the wrench accounting for joint DOF
  const VectorXd jwrench_j = J_dof*jwrench_j_tilde;

  //  Vector from joint to distal plate in joint coordinates
  //  As we have the transform from distal plate to joint origin, computing
  //  the inverse we obtain the transform from the joint to the distal plate origin
  //  And then I extract the translation part.
  Vector3d jp_dp = origin.inverse().translation();

  //  Update Adjoint tranformation
  Adg_jd.block<3,3>(0, 0) = wRj;
  Adg_jd.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Adg_jd.block<3,3>(3, 0) = -wRj*hat(jp_dp);
  Adg_jd.block<3,3>(3, 3) =  wRj;

  //  Again shifting law computing wrench at the distal plate
  VectorXd wwrench_dp = Adg_jd*jwrench_j;

  return wwrench_dp;


}
const Eigen::Vector3d PrismaticJoint::GetJointWrenchConstrain(const Eigen::VectorXd &rod_tip_wrench, const Eigen::Affine3d &distal_plate_transf) const
{
  //  Vector from attach point to joint origin
  Vector3d ap_aj = attach_point.inverse().translation();

  //  Get the Orientation of the joint frame
  Matrix3d wRj = (distal_plate_transf*origin).linear();

  Matrix<double, 6, 6> Ad_aj;
  Ad_aj.block<3,3>(0, 0) = wRj.transpose();
  Ad_aj.block<3,3>(0, 3) = Eigen::Matrix3d::Zero();
  Ad_aj.block<3,3>(3, 0) = -wRj.transpose()*hat(ap_aj);
  Ad_aj.block<3,3>(3, 3) = wRj.transpose();

  //  Wrench at joint origin in joint coordinates
  VectorXd jwrench_j = Ad_aj*rod_tip_wrench;

  Matrix<double, 3, 6> Jcnt;
  Jcnt.block<3,3>(0, 0) = Scaling(translation_axis);
  Jcnt.block<3,3>(0, 3) = Matrix3d::Zero();

  Vector3d jj_cnstr = Jcnt*jwrench_j;
  Vector3d wj_cnstr = wRj*jj_cnstr;

  return wj_cnstr;


}



} //  END namespace cpr_geometry
