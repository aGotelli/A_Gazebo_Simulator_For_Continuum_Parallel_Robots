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




#ifndef CPR_PLANAR_PHYSICS_PLUGIN_H
#define CPR_PLANAR_PHYSICS_PLUGIN_H



#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Collision.hh>
//#include <gazebo/physics/PhysicsIface.hh>

//#include <gazebo/physics/World.hh>
//#include <gazebo/physics/PhysicsEngine.hh>

#include <gazebo/physics/JointController.hh>


#include <cpr_planar_messages/JointsControl.h>
#include <geometry_msgs/Wrench.h>


#include "cpr_planar_physics/cpr_planar_physics.h"

#include "cpr_planar_messages/DGMParameters.h"
#include "cpr_planar_messages/IGMParameters.h"

#include <std_msgs/Float32MultiArray.h>

#include "cpr_planar_messages/GuessVectorHandling.h"


using ignition::math::Vector3d;

namespace gazebo
{

class RobotModelPlugin : public ModelPlugin
{
public:
    RobotModelPlugin()=default;
    ~RobotModelPlugin()override=default;

    void Load(physics::ModelPtr distal_plate, sdf::ElementPtr sdf)override;
    void Update();

protected:

    unsigned int number_of_limbs_;

    std::string limb_prefix_;

    std::vector<cpr_planar_geometry::Limb> robot_limbs_;

    std::vector<boost::shared_ptr<gazebo::physics::JointController>> limbs_controllers_;
    std::vector<double> joints_values_;

    ros::Subscriber limbs_joints_values_;
    std::map<unsigned int, float> obtained_joints_values_;
    ros::NodeHandle nh_;
    void JointCallback(const cpr_planar_messages::JointsControl::ConstPtr msg)
    {
      for(const auto &joint : msg->joints)
        obtained_joints_values_[joint.id] = joint.value;
    }

    ros::Subscriber read_wrench;
    physics_core::Wrench wrench_;
    void WrenchCallback(const geometry_msgs::Wrench::ConstPtr wrench)
    {
      wrench_.fx = wrench->force.x;
      wrench_.fy = wrench->force.y;
      wrench_.mz = wrench->torque.z;
    }

//    Eigen::Affine2d previous_distal_plate_pose { Eigen::Affine2d::Identity() };
    Eigen::Affine2d distal_plate_pose { Eigen::Affine2d::Identity() };


    physics_core::RobotPhysics robot_model_;

    physics::ModelPtr distal_plate_model_;
    physics::LinkPtr distal_plate_;

    physics::WorldPtr world_;

    event::ConnectionPtr update_event_;


    void ParseLimb(const std::string &_limb_name);

    const cpr_planar_geometry::Rod DefineRodFromParameters() const;





    Eigen::Affine2d ToAffineTransformation(const ignition::math::Pose3d &pose) const;
    ignition::math::Pose3d ToPose3d(const Eigen::Affine2d  &pose) const;

    void DefineJointProperties(const ignition::math::Pose3d &_joint_origin,
                               const ignition::math::Pose3d &_attach_point_origin,
                               const physics::JointPtr &_joint_ptr,
                               std::shared_ptr<cpr_planar_geometry::AbstractJoint> &_joint_to_define) const;



    ros::Subscriber igm_listener;
    void IGMCallBack(const cpr_planar_messages::IGMParameters::ConstPtr &igm_parameters)
    {
      //  Copy the pose
      distal_plate_pose = Eigen::Affine2d::Identity();
      distal_plate_pose.translation() << igm_parameters->pose.x, igm_parameters->pose.y;
      distal_plate_pose.linear() = Eigen::Rotation2D<double>(igm_parameters->pose.theta).toRotationMatrix();

      //  Copy the wrench
      wrench_.fx = static_cast<double>( igm_parameters->wrench.fx );
      wrench_.fy = static_cast<double>( igm_parameters->wrench.fy );
      wrench_.mz = static_cast<double>( igm_parameters->wrench.mz );

      robot_model_.SolveIGM(distal_plate_pose, joints_values_, wrench_);

//      distal_plate_pose = current_distal_plate_pose;

//      for(unsigned int i=0; i<number_of_limbs_; i++)
//        limbs_controllers_[i]->SetJointPosition(limb_prefix_+std::to_string(i+1)+"::base_joint",joints_values_[i]);
//      return;
    }

    ros::Subscriber dgm_listener;
    void DGMCallBack(const cpr_planar_messages::DGMParameters::ConstPtr &dgm_parameters)
    {
//      for(const auto &[joint_id, joint_value] : obtained_joints_values_)
//        joints_values_[joint_id] = static_cast<double>(joint_value);
//      Eigen::Affine2d current_distal_plate_pose = Eigen::Affine2d::Identity();

      joints_values_ = std::vector<double>(dgm_parameters->joints_values.begin(),
                                           dgm_parameters->joints_values.end() );

      //  Copy the wrench
      wrench_.fx = static_cast<double>( dgm_parameters->wrench.fx );
      wrench_.fy = static_cast<double>( dgm_parameters->wrench.fy );
      wrench_.mz = static_cast<double>( dgm_parameters->wrench.mz );

      robot_model_.SolveDGM(distal_plate_pose, joints_values_, wrench_);

//      previous_distal_plate_pose = current_distal_plate_pose;

      //  Move distal plate to the solution
//      distal_plate_->SetWorldPose(ToPose3d(distal_plate_pose));
    }


    ros::Publisher joints_values_publisher;

    void StreamJointsValues();

    ros::ServiceServer guess_vector_services;
    bool GuessServicesCallback(cpr_planar_messages::GuessVectorHandling::Request  &request,
                               cpr_planar_messages::GuessVectorHandling::Response &response);

};

GZ_REGISTER_MODEL_PLUGIN(RobotModelPlugin)
} //  END namespace gazebo


#endif // CPR_PLANAR_PHYSICS_PLUGIN_H
