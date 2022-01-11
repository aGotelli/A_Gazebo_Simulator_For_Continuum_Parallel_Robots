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




#ifndef CPR_PHYSICS_PLUGIN_H
#define CPR_PHYSICS_PLUGIN_H



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


#include <cpr_messages/DGMParams.h>
#include <cpr_messages/IGMParams.h>

#include <cpr_messages/GuessVectorHandling.h>

#include <cpr_messages/JointsControl.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Pose.h>

#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>

#include <std_msgs/Float32MultiArray.h>


#include "cpr_physics/cpr_physics.h"

#include "cpr_messages/RemoveRobot.h"

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

    std::vector<cpr_geometry::Limb> robot_limbs_;

    std::vector<boost::shared_ptr<gazebo::physics::JointController>> limbs_controllers_;
    std::vector<float> joints_values_;


//    std::map<unsigned int, float> obtained_joints_values_;
    ros::NodeHandle nh_;

    physics_core::Wrench wrench_;

    Eigen::Affine3d final_distal_plate_pose { Eigen::Affine3d::Identity() };


    physics_core::RobotPhysics robot_model_;

    physics::ModelPtr distal_plate_model_;
    physics::LinkPtr distal_plate_;

    physics::WorldPtr world_;

    event::ConnectionPtr update_event_;


    void ParseLimb(const std::string &_limb_name);

    const cpr_geometry::Rod DefineRodFromParameters() const;


    Eigen::Affine3d ToAffineTransformation(const ignition::math::Pose3d &pose) const;
    ignition::math::Pose3d ToPose3d(const Eigen::Affine3d  &pose) const;

    void DefineJointProperties(const ignition::math::Pose3d &_joint_origin,
                               const ignition::math::Pose3d &_attach_point_origin,
                               const physics::JointPtr &_joint_ptr,
                               std::shared_ptr<cpr_geometry::AbstractJoint> &_joint_to_define) const;

    ros::Subscriber controller_wrench_client_;
    physics_core::Wrench controller_wrench_;
    void ControllerWrenchCallBack(const geometry_msgs::Wrench::ConstPtr &wrench_msg);


    ros::Subscriber dgm_parameters_listener;
    void DGMParametersCallback(const cpr_messages::DGMParams::ConstPtr &dgm_msg);


    ros::Subscriber igm_parameters_listener;
    void IGMParametersCallback(const cpr_messages::IGMParams::ConstPtr &igm_msg);


    bool RemoveRobotCallBack(std_srvs::Empty::Request  &,
                             std_srvs::Empty::Response &);
    ros::ServiceServer remove_robot_service;

    ros::ServiceServer guess_vector_services;
    bool GuessServicesCallback(cpr_messages::GuessVectorHandling::Request  &request,
                               cpr_messages::GuessVectorHandling::Response &response);

    ros::Publisher joint_values_publisher;

    bool first_iter {true};

    void StreamJointsValues();
};

GZ_REGISTER_MODEL_PLUGIN(RobotModelPlugin)
} //  END namespace gazebo


#endif // CPR_PHYSICS_PLUGIN_H
