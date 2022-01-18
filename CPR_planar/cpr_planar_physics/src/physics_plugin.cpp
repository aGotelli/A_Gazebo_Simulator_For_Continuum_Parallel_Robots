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




#include "cpr_planar_physics/cpr_planar_physics_plugin.h"


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

  //  Reset the world for good practise
  world_->Reset();

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


//  const Eigen::Affine2d distal_plate_pose = ToAffineTransformation(distal_plate->WorldPose());
  distal_plate_pose = ToAffineTransformation(distal_plate->WorldPose());
  std::vector<double> initial_guess;

  if(nh_.getParam(/*"/"+robot_name+"_settings*/"/initial_guess", initial_guess))
    cout << "Using user defined initial guess vector" << endl;


  //  Initialize the robot model
  robot_model_.InitializeRobotModel(distal_plate_pose, robot_limbs_, initial_guess, joints_values_);

  //  Initialize subscribers for getting commands from the GUI
  limbs_joints_values_ = nh_.subscribe<cpr_planar_messages::JointsControl>("joints_control",1, boost::bind(&RobotModelPlugin::JointCallback, this, _1));
  read_wrench = nh_.subscribe<geometry_msgs::Wrench>("applied_wrench", 1, boost::bind(&RobotModelPlugin::WrenchCallback, this, _1));


  igm_listener = nh_.subscribe<cpr_planar_messages::IGMParameters>("/igm_parameters", 1, boost::bind(&RobotModelPlugin::IGMCallBack, this, _1));
  dgm_listener = nh_.subscribe<cpr_planar_messages::DGMParameters>("/dgm_parameters", 1, boost::bind(&RobotModelPlugin::DGMCallBack, this, _1));

  joints_values_publisher= nh_.advertise<std_msgs::Float32MultiArray>("/joint_values", 1);

  guess_vector_services = nh_.advertiseService("/parameters_vector_services", &RobotModelPlugin::GuessServicesCallback, this);

  //  Connect update function to each Gazebo iteration
  update_event_ = event::Events::ConnectWorldUpdateBegin(std::bind(&RobotModelPlugin::Update, this));


  cout << "Setup finished" << endl;
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
  std::shared_ptr<cpr_planar_geometry::AbstractJoint> base_joint;
  std::shared_ptr<cpr_planar_geometry::AbstractJoint> distal_plate_joint;


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
  const cpr_planar_geometry::Rod rod = DefineRodFromParameters();

  //  Now insert the limb in the robot limbs vector
  robot_limbs_.push_back( cpr_planar_geometry::Limb(base_joint, distal_plate_joint, rod,
                                               nh_.param<bool>("/"+limb_model->GetName()+"/is_extendible", false)) );

  limbs_controllers_.push_back( limb_model->GetJointController() );

  //  Show that everything worked fine
  cout << "Initialized " << limb_model->GetName() << endl;
}

const cpr_planar_geometry::Rod RobotModelPlugin::DefineRodFromParameters() const
{
  // If no definition of rod, then initialize as default
  if(!nh_.hasParam("rod"))
    return cpr_planar_geometry::Rod();
  else return cpr_planar_geometry::Rod(nh_.param("rod/E", 200e9),
                                  nh_.param("rod/G", 80e9),
                                  nh_.param("rod/radius", 1e-3),
                                  nh_.param("rod/L", 1.0),
                                  nh_.param("rod/ds", 5e-3));
}

void RobotModelPlugin::DefineJointProperties(const ignition::math::Pose3d &_joint_origin,
                                             const ignition::math::Pose3d &_attach_point_origin,
                                             const physics::JointPtr &_joint_ptr,
                                             std::shared_ptr<cpr_planar_geometry::AbstractJoint> &_joint_to_define) const
{
  //  Initialize the joint origin
  const Eigen::Affine2d joint_origin = ToAffineTransformation( _joint_origin );

  //  Now initialize the attach point
  const Eigen::Affine2d attach_point_origin = ToAffineTransformation( _attach_point_origin );

  //  Redefine the entity depending on the joint type
  if(_joint_ptr->HasType(gazebo::physics::Base::EntityType::HINGE2_JOINT) ||
     _joint_ptr->HasType(gazebo::physics::Base::EntityType::HINGE_JOINT)) {
    cout << _joint_ptr->GetName() << " is a revolute joint" << endl;
    _joint_to_define = std::make_shared<cpr_planar_geometry::RevoluteJoint>(joint_origin, attach_point_origin);
  }

  if(_joint_ptr->HasType(gazebo::physics::Base::EntityType::SLIDER_JOINT)) {
    cout << _joint_ptr->GetName() << " is a prismatic joint" << endl;
    _joint_to_define = std::make_shared<cpr_planar_geometry::PrismaticJoint>(joint_origin, attach_point_origin);
  }

  if(_joint_ptr->HasType(gazebo::physics::Base::EntityType::FIXED_JOINT)) {
    cout << _joint_ptr->GetName() << " is a fixed joint" << endl;
    _joint_to_define = std::make_shared<cpr_planar_geometry::FixedJoint>(joint_origin*attach_point_origin);
  }
}


Eigen::Affine2d RobotModelPlugin::ToAffineTransformation(const ignition::math::Pose3d &pose) const
{
  //  Initialize the transform
  Eigen::Affine2d transform = Eigen::Affine2d::Identity();

  //  Define the position
  transform.translation() = Eigen::Vector2d(pose.X(), pose.Y());

  //  Obtain the Quaternion
  const Eigen::Quaternion<double> q(pose.Rot().W(), pose.Rot().X(),
                                    pose.Rot().Y(), pose.Rot().Z());

  //  Define the orientation
  transform.linear() = q.toRotationMatrix().block<2,2>(0,0);

  return transform;

}

ignition::math::Pose3d RobotModelPlugin::ToPose3d(const Eigen::Affine2d  &pose) const
{
  //  Initialize the transform
  ignition::math::Pose3d transform;

  //  Define the position
  transform.Pos() = ignition::math::Vector3d(pose.translation().x(), pose.translation().y(), 0.05);

  //  Define rotation matrix
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
  R.block<2,2>(0,0) = pose.linear().matrix();

  //  Obtain the Quaternion
  const Eigen::Quaternion<double> q(R);

  //  Define the orientation
  transform.Rot() = ignition::math::Quaterniond(q.w(), q.x(), q.y(), q.z());

  return transform;

}

void RobotModelPlugin::Update()
{
  //  Get values from callback
  ros::spinOnce();

  //  Move the joints at the given values
  for(unsigned int i=0; i<number_of_limbs_; i++)
    limbs_controllers_[i]->SetJointPosition(limb_prefix_+std::to_string(i+1)+"::base_joint",joints_values_[i]);

  distal_plate_->SetWorldPose(ToPose3d(distal_plate_pose));

  StreamJointsValues();
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
  joints_values_publisher.publish(joint_values);
}

bool RobotModelPlugin::GuessServicesCallback(cpr_planar_messages::GuessVectorHandling::Request  &request,
                                             cpr_planar_messages::GuessVectorHandling::Response &response)
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

  Eigen::Affine2d dp_pose = Eigen::Affine2d::Identity();
  std::vector<float> joint_values;

  if(robot_model_.SetGuessVector(given_guess, dp_pose, joint_values)) {
    // Update local copy of the distal plate
    distal_plate_pose = dp_pose;

    //  Update the joint values
    joints_values_ = std::vector<double>(joint_values.begin(), joint_values.end());

    return true;
  }

  return false;

}


} //  END namespace gazebo

