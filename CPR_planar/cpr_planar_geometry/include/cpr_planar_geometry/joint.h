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


namespace cpr_planar_geometry {



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
    virtual inline const Eigen::Affine2d GetRodInitialFrame(const double& joint_value) const=0;

  /*!
     * \brief evaluatePositionError Compute the position error from the
     * \param distal_plate_transf
     * \param rod_tip_transf
     * \return
     */
    virtual inline const Eigen::Vector2d evaluatePositionError(const Eigen::Affine2d& distal_plate_transf,
                                                               const Eigen::Affine2d& rod_tip_transf) const=0;

    virtual inline double evaluateAngleError(const Eigen::Affine2d& distal_plate_transf,
                                             const Eigen::Affine2d& rod_tip_transf) const=0;

    virtual inline const Eigen::Vector3d ComputeAppliedWrench(const Eigen::Affine2d& distal_plate_transf,
                                                              const Eigen::Affine2d& rod_tip_pose,
                                                              const Eigen::Vector3d& rod_tip_wrench) const{return Eigen::Vector3d::Zero();}

   [[deprecated("This function has a bug")]]virtual inline const Eigen::Vector3d ComputeAppliedWrench(const Eigen::Affine2d& distal_plate_transf,
                                                            const Eigen::Vector3d& rod_tip_wrench) const{return Eigen::Vector3d::Zero();}

    virtual inline double GetWrenchConstrain(const Eigen::Vector3d &rod_tip_wrench, const Eigen::Affine2d &distal_plate_transf) const=0;

};





class FixedJoint : public AbstractJoint {
public:

    FixedJoint()=default;
    FixedJoint(const Eigen::Affine2d& origin_, const bool verbose=false);

    const Eigen::Affine2d GetRodInitialFrame(const double& =0) const override;


    const Eigen::Vector2d evaluatePositionError(const Eigen::Affine2d& distal_plate_transf,
                                                const Eigen::Affine2d& rod_tip_transf) const override;

    double evaluateAngleError(const Eigen::Affine2d& distal_plate_transf,
                              const Eigen::Affine2d& rod_tip_transf) const override;

    const Eigen::Vector3d ComputeAppliedWrench(const Eigen::Affine2d& distal_plate_transf,
                                               const Eigen::Affine2d& rod_tip_pose,
                                               const Eigen::Vector3d &rod_tip_wrench) const override;

    double GetWrenchConstrain(const Eigen::Vector3d&, const Eigen::Affine2d&) const override {return 0;}


private:

    //  The origin of the joint wrt the body to which is attached
    //  Global coordinates for the base joint and local for the distal plate joint
    const Eigen::Affine2d origin { Eigen::Affine2d::Identity() };
};





class RevoluteJoint : public AbstractJoint {
public:

    RevoluteJoint()=default;

    [[deprecated("Use the one which uses the Affine2d transform")]]
    RevoluteJoint(const Eigen::Vector2d origin_, const Eigen::Affine2d attach_point);

    RevoluteJoint(const Eigen::Affine2d origin_, const Eigen::Affine2d attach_point_, const bool verbose=false) : origin(origin_.translation()), attach_point(attach_point_)
    {
      if(verbose){
        std::cout << "Defined joint with origin : " << std::endl;
        std::cout << origin_.matrix() << std::endl;
        std::cout << "And attach point : " << std::endl;
        std::cout << attach_point_.matrix() << std::endl;
        std::cout << std::endl;
      }
    }

    const Eigen::Affine2d GetRodInitialFrame(const double& joint_value) const override;


    const Eigen::Vector2d evaluatePositionError(const Eigen::Affine2d& distal_plate_transf,
                                                const Eigen::Affine2d& rod_tip_transf) const override;

    double evaluateAngleError(const Eigen::Affine2d& /*distal_plate_transf*/,
                              const Eigen::Affine2d& /*rod_tip_transf*/) const override {return 0.0;}

    const Eigen::Vector3d ComputeAppliedWrench(const Eigen::Affine2d& distal_plate_transf,
                                               const Eigen::Affine2d& rod_tip_pose,
                                               const Eigen::Vector3d &rod_tip_wrench) const override;

    double GetWrenchConstrain(const Eigen::Vector3d&, const Eigen::Affine2d&) const override;


private:
    //  The origin of the joint wrt the body to which is attached
    //  Global coordinates for the base joint and local for the distal plate joint
    const Eigen::Vector2d origin { Eigen::Vector2d::Zero() };

    //  The point where the rod is attached expressed in the joint frame
    const Eigen::Affine2d attach_point { Eigen::Affine2d::Identity() };

    mutable double moment_at_joint { 0.0 };



};




class PrismaticJoint : public AbstractJoint {
public:

    PrismaticJoint()=default;

    PrismaticJoint(const Eigen::Affine2d origin_, const Eigen::Affine2d attach_point_, const bool verbose=false);

    const Eigen::Affine2d GetRodInitialFrame(const double& joint_value) const override;

    const Eigen::Vector2d evaluatePositionError(const Eigen::Affine2d& distal_plate_transf,
                                                const Eigen::Affine2d& rod_tip_transf) const override;

    double evaluateAngleError(const Eigen::Affine2d& distal_plate_transf,
                              const Eigen::Affine2d& rod_tip_transf) const override;

    [[deprecated("Need to correct the wrench computations")]]const Eigen::Vector3d ComputeAppliedWrench(const Eigen::Affine2d& distal_plate_transf,
                                               const Eigen::Vector3d &rod_tip_wrench) const override;

    [[deprecated("Need to correct the wrench computations")]]double GetWrenchConstrain(const Eigen::Vector3d& rod_tip_wrench, const Eigen::Affine2d& distal_plate_transf) const override;

private:

    //  The origin of the joint wrt the body to which is attached
    //  Global coordinates for the base joint and local for the distal plate joint
    const Eigen::Affine2d origin { Eigen::Affine2d::Identity() };

    //  The point where the rod is attached expressed in the joint frame
    const Eigen::Affine2d attach_point { Eigen::Affine2d::Identity() };

    //  The orientation of the joint wrt the world frame
    const Eigen::Rotation2D<double> orientation;

    //  A deficent Identity matrix to account for the DoF when computing the error
    const Eigen::Matrix2d I_tilde { Eigen::DiagonalMatrix<double, 2>(0, 1) };

    inline const Eigen::Affine2d GetAttachPointWrtWorld(const Eigen::Affine2d& distal_plate_transf) const
    {
      return distal_plate_transf*origin*attach_point;
    }


};


} //  END namespace cpr_planar_geometry





#endif // JOINT_H
