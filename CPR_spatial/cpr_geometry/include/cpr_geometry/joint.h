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

#ifndef JOINT_H
#define JOINT_H

#include <Eigen/Dense>
#include <iostream>

#include "cpr_geometry/skew_operations.h"



namespace cpr_geometry {





/*!
 * \brief The AbstractJoint class serves as interface for creatin a routine
 *
 * In order to solve the fiven problem, some calculations have to be performed for
 * every joint in the robot. However, these calculations depends on the joint type.
 * Whith this abstract class I create an interface to perform a given order of
 * operation to a generic joint. The correct operations will then be performed
 * depending on the implementation that will be used.
 */
class AbstractJoint {
public:
    virtual ~AbstractJoint()=default;

  /*!
     * \brief GetRodInitialFrame gives the 2D pose (wrt the reference frame) of first frame on its centerline (s=0)
     * \param joint_value the value for the joint that will be used to compute the rigid transformation
     * \return the rigid transformation from the reference frame to the first frame of the rod (at s=0)
     */
    virtual inline const Eigen::Affine3d GetRodInitialFrame(const double& joint_value) const=0;

  /*!
     * \brief evaluatePositionError Compute the position error from the
     * \param distal_plate_transf
     * \param rod_tip_transf
     * \return
     */
    virtual inline const Eigen::Vector3d evaluatePositionError(const Eigen::Affine3d& distal_plate_transf,
                                                               const Eigen::Affine3d& rod_tip_transf) const=0;

    virtual inline const Eigen::Vector3d evaluateOrientationError(const Eigen::Affine3d& distal_plate_transf,
                                                                  const Eigen::Affine3d& rod_tip_transf) const=0;

    virtual inline const Eigen::VectorXd ComputeAppliedWrench(const Eigen::Affine3d &distal_plate_transf,
                                                              const Eigen::VectorXd &rod_tip_wrench,
                                                              const Eigen::Affine3d &rod_tip_transf) const=0;

    virtual inline const Eigen::Vector3d GetJointWrenchConstrain(const Eigen::VectorXd &rod_tip_wrench,
                                                                 const Eigen::Affine3d &distal_plate_transf) const=0;
};





class FixedJoint : public AbstractJoint {
public:

    FixedJoint()=default;
    FixedJoint(const Eigen::Affine3d &origin_, const bool verbose=false);

    FixedJoint(const Eigen::Affine3d &origin_, const Eigen::Affine3d &, const bool verbose=false) : FixedJoint(origin_, verbose){}


    const Eigen::Affine3d GetRodInitialFrame(const double& =0) const override;


    const Eigen::Vector3d evaluatePositionError(const Eigen::Affine3d& distal_plate_transf,
                                                const Eigen::Affine3d& rod_tip_transf) const override;

    const Eigen::Vector3d evaluateOrientationError(const Eigen::Affine3d& distal_plate_transf,
                                                   const Eigen::Affine3d& rod_tip_transf) const override;

    const Eigen::VectorXd ComputeAppliedWrench(const Eigen::Affine3d &distal_plate_transf,
                                               const Eigen::VectorXd &rod_tip_wrench,
                                               const Eigen::Affine3d &rod_tip_transf) const override;

    const Eigen::Vector3d GetJointWrenchConstrain(const Eigen::VectorXd&, const Eigen::Affine3d&) const override {return Eigen::Vector3d::Zero();}

private:

    //  The origin of the joint wrt the body to which is attached
    //  Global coordinates for the base joint and local for the distal plate joint
    const Eigen::Affine3d origin { Eigen::Affine3d::Identity() };

    //  Adjoint transformations to apply shiftiing law to a wrench
    //  From rod to joint
    mutable Eigen::Matrix<double, 6, 6> Adg_rj;
    //  From joint to distal plate
    mutable Eigen::Matrix<double, 6, 6> Adg_jd;
};





class RevoluteJoint : public AbstractJoint {
public:

    RevoluteJoint()=default;


    RevoluteJoint(const Eigen::Affine3d &origin_,
                  const Eigen::Affine3d &attach_point_,
                  const Eigen::Vector3d &rotation_axis_=Eigen::Vector3d::UnitZ(),
                  const bool verbose=false)
                  : origin(origin_),
                    attach_point(attach_point_),
                    rotation_axis(rotation_axis_)
    {

      J_dof.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
      J_dof.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
      J_dof.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
      J_dof.block<3,3>(3,3) = Eigen::Matrix3d::Identity() - Eigen::Scaling(this->rotation_axis).toDenseMatrix();

      J_cnstr.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
      J_cnstr.block<3,3>(0,3) = Eigen::Scaling(this->rotation_axis).toDenseMatrix();


      if(verbose){
        std::cout << "Defined joint with origin : " << std::endl;
        std::cout << origin_.matrix() << std::endl;
        std::cout << "And attach point : " << std::endl;
        std::cout << attach_point_.matrix() << std::endl;
        std::cout << "And rotation axis : " << rotation_axis.transpose() << std::endl;
        std::cout << "And dof correction matrix : " << std::endl;
        std::cout << J_dof << std::endl;
        std::cout << std::endl;
      }

      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //          ADD WARN ABOUT UNITARY AXIS INITIALIZATION
    }

    const Eigen::Affine3d GetRodInitialFrame(const double& joint_value) const override;



    const Eigen::Vector3d evaluatePositionError(const Eigen::Affine3d &distal_plate_transf,
                                                const Eigen::Affine3d &rod_tip_transf) const override;

    const Eigen::Vector3d evaluateOrientationError(const Eigen::Affine3d &distal_plate_transf,
                                                   const Eigen::Affine3d &rod_tip_transf) const override;

    const Eigen::VectorXd ComputeAppliedWrench(const Eigen::Affine3d &distal_plate_transf,
                                               const Eigen::VectorXd &rod_tip_wrench,
                                               const Eigen::Affine3d &rod_tip_transf) const override;

    const Eigen::Vector3d GetJointWrenchConstrain(const Eigen::VectorXd &,
                                                  const Eigen::Affine3d &distal_plate_transf) const override;

private:
    //  The origin of the joint wrt the body to which is attached
    //  Global coordinates for the base joint and local for the distal plate joint
    const Eigen::Affine3d origin { Eigen::Affine3d::Identity() };

    //  The point where the rod is attached expressed in the joint frame
    const Eigen::Affine3d attach_point { Eigen::Affine3d::Identity() };

    //  The axis of rotation of the revolute joint
    const Eigen::Vector3d rotation_axis { Eigen::Vector3d::UnitZ() };

    //  The revolute joint implies zero moment on its axis of rotation
    Eigen::Matrix<double, 6, 6> J_dof;
    Eigen::Matrix<double, 3, 6> J_cnstr;

    mutable Eigen::Vector3d jjoint_cnstr;

};



class SphericalJoint : public AbstractJoint {
public:

    SphericalJoint()=default;

    SphericalJoint(const Eigen::Affine3d &origin_,
                  const Eigen::Affine3d &attach_point_,
                  const bool verbose=false)
                  : origin(origin_),
                    attach_point(attach_point_)
    {
      J_dof.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
      J_dof.block<3,3>(0,3) = Eigen::Matrix3d::Zero();
      J_dof.block<3,3>(3,0) = Eigen::Matrix3d::Zero();
      J_dof.block<3,3>(3,3) = Eigen::Matrix3d::Zero();

      J_cnstr.block<3,3>(0,0) = Eigen::Matrix3d::Zero();
      J_cnstr.block<3,3>(0,3) = Eigen::Matrix3d::Identity();



      if(verbose){
        std::cout << "Defined spherical joint with origin : " << std::endl;
        std::cout << origin_.matrix() << std::endl;
        std::cout << "And attach point : " << std::endl;
        std::cout << attach_point_.matrix() << std::endl;
        std::cout << "And dof correction matrix : " << std::endl;
        std::cout << J_dof << std::endl;
        std::cout << std::endl;

      }

      //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      //          ADD WARN ABOUT UNITARY AXIS INITIALIZATION
    }


    const Eigen::Affine3d GetRodInitialFrame(const double& =0) const override { return Eigen::Affine3d::Identity();}


    const Eigen::Vector3d evaluatePositionError(const Eigen::Affine3d &distal_plate_transf,
                                                const Eigen::Affine3d &rod_tip_transf) const override;

    const Eigen::Vector3d evaluateOrientationError(const Eigen::Affine3d &,
                                                   const Eigen::Affine3d &) const override { return Eigen::Vector3d::Zero();}

    const Eigen::VectorXd ComputeAppliedWrench(const Eigen::Affine3d &distal_plate_transf,
                                               const Eigen::VectorXd &rod_tip_wrench,
                                               const Eigen::Affine3d &) const override;

    const Eigen::Vector3d GetJointWrenchConstrain(const Eigen::VectorXd& rod_tip_wrench, const Eigen::Affine3d &distal_plate_transf) const override;

private:

    //  The origin of the joint wrt the body to which is attached
    //  Global coordinates for the base joint and local for the distal plate joint
    const Eigen::Affine3d origin { Eigen::Affine3d::Identity() };

    //  The point where the rod is attached expressed in the joint frame
    const Eigen::Affine3d attach_point { Eigen::Affine3d::Identity() };

    //  Adjoint transformations to apply shiftiing law to a wrench
    //  From rod to joint
    mutable Eigen::Matrix<double, 6, 6> Adg_rj { Eigen::MatrixXd::Identity(6,6) };
    //  From joint to distal plate
    mutable Eigen::Matrix<double, 6, 6> Adg_jd { Eigen::MatrixXd::Identity(6,6) };

    //  The revolute joint implies zero moment on its axis of rotation
    Eigen::Matrix<double, 6, 6> J_dof;
    Eigen::Matrix<double, 3, 6> J_cnstr;

    mutable Eigen::Vector3d jjoint_cnstr;
};




class PrismaticJoint : public AbstractJoint {
public:

    PrismaticJoint()=default;

    PrismaticJoint(const Eigen::Affine3d origin_, const Eigen::Affine3d attach_point_, const bool verbose=false);

    const Eigen::Affine3d GetRodInitialFrame(const double& joint_value) const override;



    const Eigen::Vector3d evaluatePositionError(const Eigen::Affine3d &distal_plate_transf,
                                                const Eigen::Affine3d &rod_tip_transf) const override;

    const Eigen::Vector3d evaluateOrientationError(const Eigen::Affine3d& distal_plate_transf,
                                                   const Eigen::Affine3d& rod_tip_transf) const override;

    const Eigen::VectorXd ComputeAppliedWrench(const Eigen::Affine3d &distal_plate_transf,
                                               const Eigen::VectorXd &rod_tip_wrench,
                                               const Eigen::Affine3d &rod_tip_transf) const override;

    const Eigen::Vector3d GetJointWrenchConstrain(const Eigen::VectorXd &rod_tip_wrench, const Eigen::Affine3d &distal_plate_transf) const override;

private:

    //  The origin of the joint wrt the body to which is attached
    //  Global coordinates for the base joint and local for the distal plate joint
    const Eigen::Affine3d origin { Eigen::Affine3d::Identity() };

    //  The point where the rod is attached expressed in the joint frame
    const Eigen::Affine3d attach_point { Eigen::Affine3d::Identity() };

    //  The axis of translation of the prismatic joint
    const Eigen::Vector3d translation_axis { Eigen::Vector3d::UnitZ() };


    //  Adjoint transformations to apply shiftiing law to a wrench
    //  From rod to joint
    mutable Eigen::Matrix<double, 6, 6> Adg_rj;
    //  From joint to distal plate
    mutable Eigen::Matrix<double, 6, 6> Adg_jd;

    //  The revolute joint implies zero moment on its axis of rotation
    mutable Eigen::Matrix<double, 6, 6> J_dof;




};


} //  END namespace cpr_geometry





#endif // JOINT_H
