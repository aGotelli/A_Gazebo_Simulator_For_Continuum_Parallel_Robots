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




#include "cpr_physics/cpr_physics_plugin.h"


#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/SetLinkStateRequest.h>
#include <gazebo_msgs/SetLinkStateResponse.h>


template <typename T>
inline T ReadFromSDF(sdf::ElementPtr sdf, std::string key, T fallback)
{
  if(sdf->HasElement(key))
    return sdf->Get<T>(key);
  return fallback;
}



namespace gazebo
{

using namespace std;
using namespace physics;
void RobotModelPlugin::Load(physics::ModelPtr distal_plate, sdf::ElementPtr sdf)
{
  cout << endl;
  cout << endl;
  cout << endl;
  cout << "Loading robot physics and geometrical quantities..." << std::endl;
  cout << endl;
  cout << endl;
  cout << endl;


  //  Set the robot number of limbs from the sdf file
  number_of_limbs_ = ReadFromSDF<unsigned int>(sdf, "number_of_limbs", 0);

  //  Reserve space in the vector of robot limbs
  robot_limbs_.reserve(number_of_limbs_);

  //  Get the instance of the world
  world_ = distal_plate->GetWorld();

  //  Store local instance of the distal plate pose
  distal_plate_model_ = distal_plate;
  distal_plate_ = distal_plate_model_->GetLink("distal_plate");

  cout << "models list: " << endl;
  for(const auto& model_ : world_->Models()) {
    cout << "   " << model_->GetName() << endl;
  }


  //  Get limb prefix
  nh_.param<std::string>("/limb_name_prefix", limb_prefix_, "");

  //  Get robot name
  const std::string robot_name = [&]() { std::string robot_name_;
                                         nh_.param<std::string>("/robot_name", robot_name_, "");
                                         return robot_name_; } ();

  //  Parse and initialize every limb it is already spawned
  for(unsigned int i=0; i<number_of_limbs_; i++) {
    //  Current ID
    const std::string limb_name = limb_prefix_+std::to_string(i+1);

    //  Parse the limb (if it exists)
    ParseLimb(limb_name);
  }




  //  Get the position of the distal plate model
  Eigen::Affine3d distal_plate_pose = ToAffineTransformation( distal_plate_model_->GetLink("distal_plate")->WorldPose() );
  final_distal_plate_pose = distal_plate_pose;
  std::vector<double> initial_guess;

  if(nh_.getParam("/initial_guess", initial_guess))
    cout << "Using user defined initial guess vector" << endl;


  ignition::math::Vector3d force = (distal_plate_->GetInertial()->Mass() +
                                   number_of_limbs_*distal_plate_model_->GetLink("limb1_joint_body")->GetInertial()->Mass() +
                                   number_of_limbs_*distal_plate_model_->GetLink("limb1_attach_point")->GetInertial()->Mass() )* world_->Gravity();

  physics_core::Wrench initial_wrench;
  initial_wrench.force = Eigen::Vector3d(force.X(), force.Y(), force.Z());


  std::vector<double> underactuation_vector(6);
  nh_.param<std::vector<double>>("/underactuation_vector", underactuation_vector, {1, 1, 1, 1, 1, 1});



//  Eigen::Map<Eigen::MatrixXd> undearctuation_correction_matrix(underactuation_vector.data(), 6, 6);
  Eigen::MatrixXd undearctuation_correction_matrix(6, 6);
  undearctuation_correction_matrix.block<3, 3>(0, 0) = Eigen::Matrix3d::Zero();
  undearctuation_correction_matrix.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
  undearctuation_correction_matrix.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
  undearctuation_correction_matrix.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();

  for(unsigned int i=0; i<6; i++)
    undearctuation_correction_matrix(i, i) = 100*underactuation_vector[i];



//  //  Initialize the robot model
  robot_model_.InitializeRobotModel(distal_plate_pose,
                                    robot_limbs_,
                                    initial_guess,
                                    joints_values_,
                                    undearctuation_correction_matrix,
                                    initial_wrench);

  final_distal_plate_pose = distal_plate_pose;

  //  Move distal plate to the solution
  distal_plate_->SetWorldPose( ToPose3d(distal_plate_pose) );



  dgm_parameters_listener = nh_.subscribe<cpr_messages::DGMParams>("/dgm_parameters", 1, boost::bind(&RobotModelPlugin::DGMParametersCallback, this, _1));
  igm_parameters_listener = nh_.subscribe<cpr_messages::IGMParams>("/igm_parameters", 1, boost::bind(&RobotModelPlugin::IGMParametersCallback, this, _1));

  controller_wrench_client_ = nh_.subscribe<geometry_msgs::Wrench>("/controller_wrench", 1, boost::bind(&RobotModelPlugin::ControllerWrenchCallBack, this, _1));



  remove_robot_service = nh_.advertiseService("/remove_robot", &RobotModelPlugin::RemoveRobotCallBack, this);

  guess_vector_services = nh_.advertiseService("/parameters_vector_services", &RobotModelPlugin::GuessServicesCallback, this);

  joint_values_publisher = nh_.advertise<std_msgs::Float32MultiArray>("/joint_values", 1);

  //  Connect update function to each Gazebo iteration
  update_event_ = event::Events::ConnectWorldUpdateBegin(std::bind(&RobotModelPlugin::Update, this));

  distal_plate_model_->SetGravityMode(false);

  cout << "Setup finished" << endl;

}

void RobotModelPlugin::Update()
{
  ros::spinOnce();

  distal_plate_->AddForce(ignition::math::Vector3(controller_wrench_.force.x(),
                                                  controller_wrench_.force.y(),
                                                  controller_wrench_.force.z()));

  distal_plate_->AddTorque(ignition::math::Vector3(controller_wrench_.torque.x(),
                                                   controller_wrench_.torque.y(),
                                                   controller_wrench_.torque.z()));


  //  Control the joints at the given values
  StreamJointsValues();

  distal_plate_->SetWorldPose( ToPose3d(final_distal_plate_pose) );



}

void RobotModelPlugin::ControllerWrenchCallBack(const geometry_msgs::Wrench::ConstPtr &wrench_msg)
{
  //  Get the controller Wrench
  controller_wrench_.force = Eigen::Vector3d(wrench_msg->force.x,
                                             wrench_msg->force.y,
                                             wrench_msg->force.z);
  controller_wrench_.torque = Eigen::Vector3d(wrench_msg->torque.x,
                                              wrench_msg->torque.y,
                                              wrench_msg->torque.z);

  //cout << "Applying the wrench : " << endl << controller_wrench_.force.x() << endl << controller_wrench_.force.y() << endl << controller_wrench_.force.z() << endl << endl << endl ;
  //  Get the current distal plate pose
  Eigen::Affine3d distal_plate_pose = ToAffineTransformation( distal_plate_model_->GetLink("distal_plate")->WorldPose() );

  //  Solve the assembly to control the joints at the given position
  robot_model_.SolveIGM(distal_plate_pose, joints_values_, controller_wrench_);


  //  Control the joints at the given values
  StreamJointsValues();

}


void RobotModelPlugin::DGMParametersCallback(const cpr_messages::DGMParams::ConstPtr &dgm_msg)
{

  if(dgm_msg->joints_values.size() != joints_values_.size())
    std::cout << "Invalid copy for joint values vectors of different sizes" << std::endl;
  else joints_values_ = dgm_msg->joints_values;

  wrench_.force = Eigen::Vector3d(dgm_msg->distal_plate_wrench.force.x,
                                  dgm_msg->distal_plate_wrench.force.y,
                                  dgm_msg->distal_plate_wrench.force.z);
  wrench_.torque = Eigen::Vector3d(dgm_msg->distal_plate_wrench.torque.x,
                                   dgm_msg->distal_plate_wrench.torque.y,
                                   dgm_msg->distal_plate_wrench.torque.z);



  //    Eigen::Affine3d distal_plate_pose = Eigen::Affine3d::Identity();
  Eigen::Affine3d distal_plate_pose = Eigen::Affine3d::Identity();

  //    robot_model_.SolveDGM(distal_plate_pose, joints_values_, wrench_);
  robot_model_.SolveDGM(distal_plate_pose, joints_values_, wrench_);

  //  Move distal plate to the solution
  distal_plate_->SetWorldPose( ToPose3d(distal_plate_pose) );

  final_distal_plate_pose = distal_plate_pose;

  //  Control the joints at the given values
  StreamJointsValues();

}

void RobotModelPlugin::IGMParametersCallback(const cpr_messages::IGMParams::ConstPtr &igm_msg)
{
//      solve_igm = true;
  Eigen::Affine3d received_dp_pose = Eigen::Affine3d::Identity();
  received_dp_pose.translation() = Eigen::Vector3d(igm_msg->distal_plate_pose.position.x,
                                                             igm_msg->distal_plate_pose.position.y,
                                                             igm_msg->distal_plate_pose.position.z);

  Eigen::Quaternion<double> q(igm_msg->distal_plate_pose.orientation.w,
                              igm_msg->distal_plate_pose.orientation.x,
                              igm_msg->distal_plate_pose.orientation.y,
                              igm_msg->distal_plate_pose.orientation.z);

  received_dp_pose.linear() = q.toRotationMatrix();


  wrench_.force = Eigen::Vector3d(igm_msg->distal_plate_wrench.force.x,
                                  igm_msg->distal_plate_wrench.force.y,
                                  igm_msg->distal_plate_wrench.force.z);
  wrench_.torque = Eigen::Vector3d(igm_msg->distal_plate_wrench.torque.x,
                                   igm_msg->distal_plate_wrench.torque.y,
                                   igm_msg->distal_plate_wrench.torque.z);

  robot_model_.SolveIGM(received_dp_pose, joints_values_, wrench_);

  //  Control the joints at the given values
  StreamJointsValues();

  //  Move distal plate to the solution
  distal_plate_->SetWorldPose( ToPose3d(received_dp_pose) );

  final_distal_plate_pose = received_dp_pose;

  cout << "Current Distal Plate Pose : " << endl << received_dp_pose.matrix() << endl << endl << endl;

}



bool RobotModelPlugin::RemoveRobotCallBack(std_srvs::Empty::Request &,
                                           std_srvs::Empty::Response &)
{
  //  Hides the lines
  robot_model_.RemoveLines();

  for(unsigned int i=1; i<=number_of_limbs_; i++) {
    ModelPtr limb_model_ptr = world_->ModelByName(limb_prefix_ + std::to_string(i));
    limb_model_ptr->Fini();
    world_->RemoveModel(limb_model_ptr);

    //  Delete the parameter telling if the limb is extensible to avoid bugs with non extendible limbs
    bool temporary_bool;
    if(nh_.getParam("/"+limb_model_ptr->GetName()+"/is_extendible", temporary_bool))
      nh_.deleteParam("/"+limb_model_ptr->GetName()+"/is_extendible");
  }

  distal_plate_model_->Fini();

  world_->RemoveModel(distal_plate_model_);

  return true;
}

bool RobotModelPlugin::GuessServicesCallback(cpr_messages::GuessVectorHandling::Request  &request,
                                             cpr_messages::GuessVectorHandling::Response &response)
{
  //  If the given vector is empty, copy the value from the current one;
  if(request.given_guess.empty()) {
    robot_model_.GetCurrentGuessVector( response.actual_guess );
    return true;
  }

  //  Else, copy the given guess vector values into the robot model

  std::vector<double> given_guess(request.given_guess.size());

  for(unsigned int i=0; i<request.given_guess.size(); i++)
    given_guess[i] = static_cast<double>( request.given_guess[i] );

  Eigen::Affine3d distal_plate_pose = Eigen::Affine3d::Identity();
  std::vector<float> joint_values;

  if(robot_model_.SetGuessVector(given_guess, distal_plate_pose, joint_values)) {
    // Update local copy of the distal plate
    final_distal_plate_pose = distal_plate_pose;

    //  Update the joint values
    joints_values_ = joint_values;

    return true;
  }

  return false;

}



void RobotModelPlugin::ParseLimb(const std::string &_limb_name)
{
  const ModelPtr limb_model = world_->ModelByName(_limb_name);

  cout << endl;
  if(limb_model == nullptr) {
    cout << _limb_name << " is not in the simulator yet." << endl;
    return;
  } else cout << "Parsing " << limb_model->GetName() << endl;


  //  Define abstract joints elements
  std::shared_ptr<cpr_geometry::AbstractJoint> base_joint;
  std::shared_ptr<cpr_geometry::AbstractJoint> distal_plate_joint;


  //  Obtain the base joint pointer
  const JointPtr ptr_base_joint = limb_model->GetJoint("base_joint");

  //  Define the base joint pose
  const ignition::math::Pose3d base_joint_origin = ptr_base_joint->WorldPose();

  //  Define the attach point pose for the base joint
  const ignition::math::Pose3d base_joint_attach_point = limb_model->GetLink("attach_point")->WorldPose() -
                                                         ptr_base_joint->WorldPose()                      ;

  //  Now define the base joint given the geometrical dimension and its type
  DefineJointProperties(base_joint_origin, base_joint_attach_point, ptr_base_joint, base_joint);



  //  Obtain the distal plate joint
  const JointPtr ptr_distal_plate_joint = distal_plate_model_->GetJoint(_limb_name+"_joint");

  //  Transform from distal plate to joint origin
  const ignition::math::Pose3d from_distal_plate_to_limb_joint = ptr_distal_plate_joint->WorldPose() -
                                                                 distal_plate_model_->GetLink("distal_plate")->WorldPose();


  //  Define the attach point pose for the distal plate joint
  const ignition::math::Pose3d distal_plate_joint_to_attach_point = distal_plate_model_->GetLink(_limb_name+"_attach_point")->WorldPose() -
                                                                    ptr_distal_plate_joint->WorldPose();

  //  Now define the base joint given the geometrical dimension and its type
  DefineJointProperties(from_distal_plate_to_limb_joint, distal_plate_joint_to_attach_point, ptr_distal_plate_joint, distal_plate_joint);

  //  Construct a rod with user parameters (if any)
  const cpr_geometry::Rod rod = DefineRodFromParameters();

  //  Now insert the limb in the robot limbs vector
  robot_limbs_.push_back( cpr_geometry::Limb(base_joint, distal_plate_joint, rod,
                                               nh_.param<bool>("/"+limb_model->GetName()+"/is_extendible", false)) );

  limbs_controllers_.push_back( limb_model->GetJointController() );


  //  Show that everything worked fine
  cout << "Initialized " << limb_model->GetName() << endl;
}

const cpr_geometry::Rod RobotModelPlugin::DefineRodFromParameters() const
{
  // If no definition of rod, then initialize as default
  if(!nh_.hasParam("rod"))
    return cpr_geometry::Rod();
  else return cpr_geometry::Rod(nh_.param("rod/E", 200e9),
                                  nh_.param("rod/G", 80e9),
                                  nh_.param("rod/radius", 1e-3),
                                  nh_.param("rod/L", 1.0),
                                  nh_.param("rod/ds", 5e-3));
}

void RobotModelPlugin::DefineJointProperties(const ignition::math::Pose3d &_joint_origin,
                                             const ignition::math::Pose3d &_attach_point_origin,
                                             const physics::JointPtr &_joint_ptr,
                                             std::shared_ptr<cpr_geometry::AbstractJoint> &_joint_to_define) const
{
  //  Initialize the joint origin
  const Eigen::Affine3d joint_origin = ToAffineTransformation( _joint_origin );

  //  Now initialize the attach point
  const Eigen::Affine3d attach_point_origin = ToAffineTransformation( _attach_point_origin );

  //  Now define the axis (usually in z)
  const Eigen::Vector3d axis = joint_origin.linear() * Eigen::Vector3d::UnitZ();

  //  Redefine the entity depending on the joint type
  if(_joint_ptr->HasType(gazebo::physics::Base::EntityType::HINGE2_JOINT) ||
     _joint_ptr->HasType(gazebo::physics::Base::EntityType::HINGE_JOINT)) {
    cout << _joint_ptr->GetName() << " is a revolute joint" << endl;
    cout << "With origin : " << endl << joint_origin.matrix() << endl;
    cout << "With axis : " << axis.transpose() << endl;
    _joint_to_define = std::make_shared<cpr_geometry::RevoluteJoint>(joint_origin, attach_point_origin, axis);
  }

  if(_joint_ptr->HasType(gazebo::physics::Base::EntityType::SLIDER_JOINT)) {
    cout << _joint_ptr->GetName() << " is a prismatic joint" << endl;
    _joint_to_define = std::make_shared<cpr_geometry::PrismaticJoint>(joint_origin, attach_point_origin);
  }

  if(_joint_ptr->HasType(gazebo::physics::Base::EntityType::FIXED_JOINT)) {
    cout << _joint_ptr->GetName() << " is a fixed joint" << endl;
    _joint_to_define = std::make_shared<cpr_geometry::FixedJoint>(joint_origin*attach_point_origin);
  }

  if(_joint_ptr->HasType(gazebo::physics::Base::EntityType::BALL_JOINT)) {
    cout << _joint_ptr->GetName() << " is a spherical joint" << endl;
    _joint_to_define = std::make_shared<cpr_geometry::SphericalJoint>(joint_origin, attach_point_origin);
  }
}


Eigen::Affine3d RobotModelPlugin::ToAffineTransformation(const ignition::math::Pose3d &pose) const
{
  //  Initialize the transform
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();

  //  Define the position
  transform.translation() = Eigen::Vector3d(pose.X(), pose.Y(), pose.Z());

  //  Obtain the Quaternion
  const Eigen::Quaternion<double> q(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());

  //  Define the orientation
  transform.linear() = q.toRotationMatrix();

  return transform;

}

ignition::math::Pose3d RobotModelPlugin::ToPose3d(const Eigen::Affine3d  &pose) const
{
  //  Initialize the transform
  ignition::math::Pose3d transform;

  //  Define the position
  transform.Pos() = ignition::math::Vector3d(pose.translation().x(),
                                             pose.translation().y(),
                                             pose.translation().z());

  //  Obtain the Quaternion
  const Eigen::Quaternion<double> q( pose.linear() );

  //  Define the orientation
  transform.Rot() = ignition::math::Quaterniond(q.w(), q.x(), q.y(), q.z());

  return transform;

}

void RobotModelPlugin::StreamJointsValues()
{
  //  Move the joints at the given values
  std_msgs::Float32MultiArray joint_values;
  joint_values.data.resize(number_of_limbs_);
  for(unsigned int i=0; i<number_of_limbs_; i++) {

    joint_values.data[i] = joints_values_[i];
    limbs_controllers_[i]->SetJointPosition(limb_prefix_ + std::to_string(i+1) + "::base_joint",
                                            static_cast<double>(joints_values_[i]));
  }
  joint_values_publisher.publish(joint_values);
}


} //  END namespace gazebo

