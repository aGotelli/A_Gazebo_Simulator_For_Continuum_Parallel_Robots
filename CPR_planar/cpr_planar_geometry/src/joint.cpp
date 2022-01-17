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



#include "cpr_planar_geometry/joint.h"

using namespace Eigen;

namespace cpr_planar_geometry{




FixedJoint::FixedJoint(const Affine2d& origin_, const bool verbose) : origin(origin_)
{
  if(verbose) {
    std::cout << "Initialized joint with origin :" << std::endl;
    std::cout << origin.matrix() << std::endl;
    std::cout << "And the corresponding wrench map :" << std::endl;
    std::cout << origin.matrix() << std::endl;
  }
}


const Affine2d FixedJoint::GetRodInitialFrame(const double&) const
{
  return origin;
}


const Vector2d FixedJoint::evaluatePositionError(const Affine2d& distal_plate_transf,
                                                        const Affine2d& rod_tip_transf) const
{
  return Vector2d( rod_tip_transf.translation() ) -
          Vector2d( (distal_plate_transf*origin ).translation() ) ;
}



double FixedJoint::evaluateAngleError(const Affine2d& distal_plate_transf,
                                      const Affine2d& rod_tip_transf) const
{
  return Rotation2D<double>( ( distal_plate_transf*origin ).linear() * rod_tip_transf.linear().transpose() ).angle();
}



const Eigen::Vector3d FixedJoint::ComputeAppliedWrench(const Affine2d& distal_plate_transf,
                                                       const Eigen::Affine2d& rod_tip_pose,
                                                       const Eigen::Vector3d &rod_tip_wrench) const
{
  //  As the rod is just attached to the distal plate, just compute the vector from the rod tip to the dp origin
  Eigen::Vector2d r = distal_plate_transf.translation() - rod_tip_pose.translation();

  return Vector3d(                rod_tip_wrench.x(),
                                  rod_tip_wrench.y(),
                                  rod_tip_wrench.z()+
                  r.x()*rod_tip_wrench.y() - r.y()*rod_tip_wrench.x());
}








RevoluteJoint::RevoluteJoint(const Vector2d origin_, const Affine2d attach_point) :
              origin( origin_ ),
              attach_point(attach_point){}


const Affine2d RevoluteJoint::GetRodInitialFrame(const double& joint_value) const
{
  return Affine2d( Translation2d(origin.x(), origin.y()) )* Affine2d( Rotation2D<double>(joint_value) )*attach_point;
}

const Eigen::Vector2d RevoluteJoint::evaluatePositionError(const Affine2d& distal_plate_transf,
                                                           const Affine2d& rod_tip_transf) const
{

  return Vector2d( distal_plate_transf*origin.homogeneous() ) -
          Vector2d( (rod_tip_transf*attach_point.inverse()).translation() );
}

const Eigen::Vector3d RevoluteJoint::ComputeAppliedWrench(const Affine2d& distal_plate_transf,
                                                          const Eigen::Affine2d& rod_tip_pose,
                                                          const Eigen::Vector3d &rod_tip_wrench) const
{
  //  Vector from the rod tip to joint origin
  Eigen::Vector2d r_jr = rod_tip_pose.translation() - Vector2d( distal_plate_transf*origin.homogeneous() );

  //  Compute the wrench that will be applied at the joint
  Vector3d wrench_at_joint(                rod_tip_wrench.x(),
                                           rod_tip_wrench.y(),
                                           rod_tip_wrench.z()+
                           r_jr.x()*rod_tip_wrench.y() - r_jr.y()*rod_tip_wrench.x() );

  //  The moment at the joint origin is an error
  moment_at_joint = wrench_at_joint.z();

  //  But set it as zero for the wrench at the dp
  wrench_at_joint.z() = 0.0;

  // Now compute the wrench that is applied at the distal plate
  Vector2d r_dpr = rod_tip_pose.translation() - distal_plate_transf.translation();

  Vector3d wrench_at_distal_plate(                   rod_tip_wrench.x(),
                                                     rod_tip_wrench.y(),
                                                     rod_tip_wrench.z()+
                                  r_dpr.x()*rod_tip_wrench.y() - r_dpr.y()*rod_tip_wrench.x() );


  return wrench_at_distal_plate;
}
//{
//  Eigen::Vector2d r = distal_plate_transf.linear()*origin;
////  std::cout << "r : " << r.transpose() << std::endl;
////  return Vector3d(                rod_tip_wrench.x(),
////                                  rod_tip_wrench.y(),
////                                  rod_tip_wrench.z()+
////                  r.x()*rod_tip_wrench.y() - r.y()*rod_tip_wrench.x());
//  return Vector3d(                rod_tip_wrench.x(),
//                                  rod_tip_wrench.y(),
//                  r.x()*rod_tip_wrench.y() - r.y()*rod_tip_wrench.x());
//}

double RevoluteJoint::GetWrenchConstrain(const Eigen::Vector3d&,
                                         const Eigen::Affine2d&) const
{
  return moment_at_joint;
}

//{
//  auto from_rod_tip_to_joint_origin = attach_point.inverse().translation();

//  const double moment_from_forces = from_rod_tip_to_joint_origin.x()*rod_tip_wrench.y() -
//                                    from_rod_tip_to_joint_origin.y()*rod_tip_wrench.x() ;
//  return rod_tip_wrench.z() + moment_from_forces;
//}










PrismaticJoint::PrismaticJoint(const Affine2d origin_, const Affine2d attach_point_, const bool verbose) :
  origin(origin_),
  attach_point(attach_point_){
  if(verbose){
    std::cout << "Intialized the prismatic joint." << std::endl;
    std::cout << "The attach point : " << attach_point.translation().transpose() << std::endl;
    std::cout << "The joint origin : " << std::endl;
    std::cout << origin.matrix() << std::endl;
    std::cout << "The attach point in local frame : " << std::endl;
    std::cout << attach_point.matrix() << std::endl;
    std::cout << "with angle : " << Eigen::Rotation2D<double>( attach_point.rotation() ).angle()*180/M_PI << std::endl;
  }
}

const Affine2d PrismaticJoint::GetRodInitialFrame(const double& joint_value) const
{
  return origin*Affine2d( Translation2d(joint_value, 0) )*attach_point;
}


const Vector2d PrismaticJoint::evaluatePositionError(const Affine2d& distal_plate_transf,
                                                   const Affine2d& rod_tip_transf) const
{
  //  Compute difference btw joint origin and rod tip position
  const Vector2d position_difference = Vector2d( rod_tip_transf.translation() )                   -
                                        GetAttachPointWrtWorld(distal_plate_transf).translation() ;

  return ( distal_plate_transf*origin ).linear()*I_tilde*( distal_plate_transf*origin ).linear().transpose()*position_difference;

}


double PrismaticJoint::evaluateAngleError(const Affine2d& distal_plate_transf,
                                          const Affine2d& rod_tip_transf) const
{
  return Rotation2D<double>( ( distal_plate_transf*origin*attach_point ).linear() * rod_tip_transf.linear().inverse() ).angle();
}

const Eigen::Vector3d PrismaticJoint::ComputeAppliedWrench(const Eigen::Affine2d& distal_plate_transf,
                                           const Eigen::Vector3d &rod_tip_wrench) const
{
  Eigen::Vector2d r = distal_plate_transf.linear()*origin.translation();

  return Vector3d(                rod_tip_wrench.x(),
                                  rod_tip_wrench.y(),
                                  rod_tip_wrench.z()+
                  r.x()*rod_tip_wrench.y() - r.y()*rod_tip_wrench.x());
}
//{
//  Eigen::Vector2d r = distal_plate_transf.linear()*origin.translation();

//  return Vector3d(                rod_tip_wrench.x(),
//                                  rod_tip_wrench.y(),
//                                  rod_tip_wrench.z()+
//                  r.x()*rod_tip_wrench.y() - r.y()*rod_tip_wrench.x());
//}

double PrismaticJoint::GetWrenchConstrain(const Eigen::Vector3d& rod_tip_wrench, const Affine2d& distal_plate_transf) const
{
  Eigen::Vector2d projected_force = ( distal_plate_transf*origin ).linear().transpose()*rod_tip_wrench.segment(0,2);
  return projected_force.x();
}



} //  END namespace cpr_planar_geometry



