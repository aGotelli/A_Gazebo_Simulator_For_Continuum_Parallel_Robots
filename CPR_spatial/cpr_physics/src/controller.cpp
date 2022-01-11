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




#include <ros/ros.h>
#include <Eigen/Dense>

#include <std_msgs/Float32.h>

#include <dynamic_reconfigure/server.h>

#include <gazebo_msgs/GetLinkState.h>
#include <gazebo_msgs/ApplyBodyWrench.h>

#include "cpr_messages/PIDControllerTuningConfig.h"
#include "cpr_messages/IGMParams.h"
#include "cpr_messages/SE3Pose.h"


#include "cpr_geometry/skew_operations.h"

#include <yaml-cpp/yaml.h>
//#include <log2plot/logger.h>
//#include <ros/package.h>

using namespace std;


Eigen::Affine3d GetDesiredPose(const double &set_point, const int controlled_dof)
{
  Eigen::Affine3d desired_pose = Eigen::Affine3d::Identity();
  Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

  //  Depending on the dof to control, define the desired pose from set point
  switch (controlled_dof) {
  case 0: // case x is controlled
    desired_pose.translation().x() = set_point;
    break;
  case 1: // case y is controlled
    desired_pose.translation().y() = set_point;
    break;
  case 2: // case z is controlled
    desired_pose.translation().z() = set_point;
    break;
  case 3: // case roll is controlled
    R = Eigen::AngleAxisd(set_point, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitZ());
    desired_pose.linear() = R;
    //LogRotationMatrix( R );
    break;
  case 4: // case pitch is controlled
    R = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(set_point, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ());
    desired_pose.linear() = R;
    //LogRotationMatrix( R );
    break;
  case 5: // case yaw is controlled
    R = Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())
        * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())
        * Eigen::AngleAxisd(set_point, Eigen::Vector3d::UnitZ());
    desired_pose.linear() = R;
    //LogRotationMatrix( R );
    break;
  }

  return desired_pose;
}


static Eigen::Affine3d desired_pose = Eigen::Affine3d::Identity();
static bool callback_executed = false;
static bool just_updated = false;
void IGMParametersCallback(const cpr_messages::IGMParams::ConstPtr &igm_msg)
{
  desired_pose.translation() = Eigen::Vector3d(igm_msg->distal_plate_pose.position.x,
                                                             igm_msg->distal_plate_pose.position.y,
                                                             igm_msg->distal_plate_pose.position.z);

  Eigen::Quaternion<double> q(igm_msg->distal_plate_pose.orientation.w,
                              igm_msg->distal_plate_pose.orientation.x,
                              igm_msg->distal_plate_pose.orientation.y,
                              igm_msg->distal_plate_pose.orientation.z);

  desired_pose.linear() = q.toRotationMatrix();

  just_updated = true;

  if(!callback_executed)
    callback_executed = true;
}

Eigen::Affine3d ToAffine3d( const geometry_msgs::Pose &pose)
{
  //  Directly compute the position
  Eigen::Affine3d eigen_pose = Eigen::Affine3d::Identity();
  eigen_pose.translation() << pose.position.x,
                              pose.position.y,
                              pose.position.z;

  //  Copy the quaternion for the  orientation
  Eigen::Quaternion<double> q(pose.orientation.w,
                              pose.orientation.x,
                              pose.orientation.y,
                              pose.orientation.z);

  //  Convert the quaternion to a rotation matrix
  eigen_pose.linear() = q.toRotationMatrix();

  return eigen_pose;
}


Eigen::VectorXd ComputeSE3Error(const Eigen::Affine3d &desired_pose, const geometry_msgs::Pose &current_pose)
{

  //  Directly compute the position error
  Eigen::Vector3d position_error(desired_pose.translation().x() - current_pose.position.x,
                                 desired_pose.translation().y() - current_pose.position.y,
                                 desired_pose.translation().z() - current_pose.position.z);

  //  Copy the quaternion for the current distal plate orientation
  Eigen::Quaternion<double> q_current(current_pose.orientation.w,
                                      current_pose.orientation.x,
                                      current_pose.orientation.y,
                                      current_pose.orientation.z);

  //  Convert the quaternion to a rotation matrix
  Eigen::Matrix3d R_current = q_current.toRotationMatrix();

  //  Obtain the desired rotation matrix
  Eigen::Matrix3d R_desired = desired_pose.linear().matrix();

  //  Compute the error in the orientation
  Eigen::Vector3d orientation_error = cpr_geometry::inv_hat(R_current.transpose()*R_desired -
                                                              R_current*R_desired.transpose());

  //  Compose the vector containing the vector in SE(3)
  Eigen::VectorXd SE3_error(6);
  SE3_error << position_error, orientation_error;
  return SE3_error;
}



Eigen::Matrix<double, 6, 6> ToGainMatrix(const double &e_x,
                                         const double &e_y,
                                         const double &e_z,
                                         const double &e_roll,
                                         const double &e_pitch,
                                         const double &e_yaw)
{
  Eigen::VectorXd proportional_gains(6);
  proportional_gains << e_x, e_y, e_z, e_roll, e_pitch, e_yaw;
  return proportional_gains.asDiagonal();
}

double LimitAngle(double angle)
{
  double corrected_angle = angle;
  if(angle > 3)
    corrected_angle = angle - M_PI;
  if(angle < -3)
    corrected_angle = angle + M_PI;

  return corrected_angle;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");
  ros::NodeHandle nh;

  const double dt = 0.001;

  ros::Subscriber igm_parameters_listener = nh.subscribe<cpr_messages::IGMParams>("/igm_parameters",1, IGMParametersCallback);

  ros::ServiceClient link_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
  gazebo_msgs::GetLinkStateRequest link_state_request;
  gazebo_msgs::GetLinkStateResponse link_state_response;
  link_state_request.link_name = "distal_plate::distal_plate";
  link_state_request.reference_frame = "world";

  ros::ServiceClient apply_wrench_client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrenchRequest apply_body_wrench_request;
  gazebo_msgs::ApplyBodyWrenchResponse apply_body_wrench_response;
  apply_body_wrench_request.body_name = "distal_plate::distal_plate";
  apply_body_wrench_request.reference_frame = "world";
  apply_body_wrench_request.duration.fromSec(dt);


  ros::Publisher distal_plate_wrench_controller = nh.advertise<geometry_msgs::Wrench>("/controller_wrench", 1);
  geometry_msgs::Wrench correction_wrench;

  //  Here the declaration of the Server element.
  dynamic_reconfigure::Server<cpr_messages::PIDControllerTuningConfig> server;

  cpr_messages::SE3Pose desired_values;
  ros::Publisher desired_values_publisher = nh.advertise<cpr_messages::SE3Pose>("/desired_values", 1);

  cpr_messages::SE3Pose current_values;
  ros::Publisher current_values_publisher = nh.advertise<cpr_messages::SE3Pose>("/current_values", 1);


  ros::Publisher time_publisher = nh.advertise<std_msgs::Float32>("/time", 1);


  double set_point = 0.0;
  ros::Subscriber set_point_listener = nh.subscribe<std_msgs::Float32>("/desired_value", 1,
                                                                            [&set_point](const std_msgs::Float32::ConstPtr &msg){
                                                                              set_point = static_cast<double>(msg->data);
                                                                            });

  double kpe_x     = 2.5;
  double kpe_y     = 2.5;
  double kpe_z     = 2.5;
  double kpe_roll  = 0.04;
  double kpe_pitch = 0.04;
  double kpe_yaw   = 0.4;

  Eigen::Matrix<double, 6, 6> Kp = 2*ToGainMatrix(kpe_x, kpe_y, kpe_z, kpe_roll, kpe_pitch, kpe_yaw);


  double kde_x     = 2.2;
  double kde_y     = 2.2;
  double kde_z     = 2.5;
  double kde_roll  = 0.035;
  double kde_pitch = 0.035;
  double kde_yaw   = 0.4;

  Eigen::Matrix<double, 6, 6> Kd = 1.8*ToGainMatrix(kde_x, kde_y, kde_z, kde_roll, kde_pitch, kde_yaw);


//  double kie_x     = 0.0;
//  double kie_y     = 0.0;
//  double kie_z     = 0.0;
//  double kie_roll  = 0.0;
//  double kie_pitch = 0.0;
//  double kie_yaw   = 0.0;
  double kie_x     = 5;
  double kie_y     = 5;
  double kie_z     = 5;
  double kie_roll  = 0.5;
  double kie_pitch = 0.5;
  double kie_yaw   = 0.5;

  Eigen::Matrix<double, 6, 6> Ki = 0.01*ToGainMatrix(kie_x, kie_y, kie_z, kie_roll, kie_pitch, kie_yaw);


  int controlled_dof = 2;
  bool start = false;
  server.setCallback( [&](const cpr_messages::PIDControllerTuningConfig &config, uint32_t){
    start = config.start;

//    Kp = ToGainMatrix(config.kde_x, config.kde_y, config.kde_z, config.kde_roll, config.kde_pitch, config.kde_yaw);

//    Kd = ToGainMatrix(config.kde_x, config.kde_y, config.kde_z, config.kde_roll, config.kde_pitch, config.kde_yaw);

//    Ki = ToGainMatrix(config.kie_x, config.kie_y, config.kie_z, config.kie_roll, config.kie_pitch, config.kie_yaw);

  } );


  Eigen::VectorXd SE3_error = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd previous_SE3_error = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd SE3_error_derivative = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd SE3_error_integral = Eigen::VectorXd::Zero(6);
  Eigen::VectorXd PID_output = Eigen::VectorXd::Zero(6);


  YAML::Emitter emitter;
  std::vector<double> z_des;
  std::vector<double> roll_des;
  std::vector<double> pitch_des;

  std::vector<double> z;
  std::vector<double> roll;
  std::vector<double> pitch;

  std::vector<double> time;

  std_msgs::Float32 time_msgs;



  ros::Rate rate(1.0/dt);
  while(ros::ok()){
    ros::spinOnce();

    time.push_back( ros::Time::now().toSec() );

    time_msgs.data = ros::Time::now().toSec();
    time_publisher.publish( time_msgs );


    if(!callback_executed /*|| !start*/) {
      cout << "Waiting..." << endl;
      PID_output = Eigen::VectorXd::Zero(6);
      SE3_error_integral = Eigen::VectorXd::Zero(6);
      cout << "Desired Pose : " << endl << GetDesiredPose(set_point, controlled_dof).matrix() << endl << endl;
      rate.sleep();
      continue;
    }

    //  Otherwise get the actual position of the distal plate
    if(!link_state_client.call(link_state_request, link_state_response)){
      //  If the call has failed just skip
      rate.sleep();
      ROS_WARN_STREAM(link_state_response.status_message);
      continue;
    }



    //  Comput the error in SE3
//    SE3_error = ComputeSE3Error(GetDesiredPose(set_point, controlled_dof),
//                                               link_state_response.link_state.pose);

    SE3_error = ComputeSE3Error(desired_pose, link_state_response.link_state.pose);

    //  Comput the integral
    SE3_error_integral += SE3_error * dt;

    //  Comput the Derivative
    SE3_error_derivative = (SE3_error - previous_SE3_error)/dt;
    previous_SE3_error = SE3_error;


    PID_output = Kp * SE3_error            +  //  P
                 Ki * SE3_error_integral   +  //  I
                 Kd * SE3_error_derivative ;  //  D


//    apply_body_wrench_request.start_time = ros::Time::now();
//    apply_body_wrench_request.wrench.force.x = PID_output[0];
//    apply_body_wrench_request.wrench.force.y = PID_output[1];
//    apply_body_wrench_request.wrench.force.z = PID_output[2];
//    apply_body_wrench_request.wrench.torque.x = PID_output[3];
//    apply_body_wrench_request.wrench.torque.y = PID_output[4];
//    apply_body_wrench_request.wrench.torque.z = PID_output[5];

//    if(!apply_wrench_client.call(apply_body_wrench_request, apply_body_wrench_response))
//      ROS_WARN_STREAM(apply_body_wrench_response.status_message);


    correction_wrench.force.x = PID_output[0];
    correction_wrench.force.y = PID_output[1];
    correction_wrench.force.z = PID_output[2];
    correction_wrench.torque.x = PID_output[3];
    correction_wrench.torque.y = PID_output[4];
    correction_wrench.torque.z = PID_output[5];

    distal_plate_wrench_controller.publish( correction_wrench );


    //  Publish the values for plotting
    desired_values.x = static_cast<float>( desired_pose.translation().x() );
    desired_values.y = static_cast<float>( desired_pose.translation().y() );
    desired_values.z = static_cast<float>( desired_pose.translation().z() );
    desired_values.roll = static_cast<float>( LimitAngle(desired_pose.linear().eulerAngles(0,1,2)[0]) );
    desired_values.pitch = static_cast<float>( LimitAngle(desired_pose.linear().eulerAngles(0,1,2)[1]) );
    desired_values.yaw = static_cast<float>( LimitAngle(desired_pose.linear().eulerAngles(0,1,2)[2]) );

    desired_values_publisher.publish( desired_values );


    Eigen::Affine3d current_pose = ToAffine3d( link_state_response.link_state.pose );


    current_values.x = static_cast<float>( current_pose.translation().x() );
    current_values.y = static_cast<float>( current_pose.translation().y() );
    current_values.z = static_cast<float>( current_pose.translation().z() );
    current_values.roll = static_cast<float>( LimitAngle(current_pose.linear().eulerAngles(0,1,2)[0]) );
    current_values.pitch = static_cast<float>( LimitAngle(current_pose.linear().eulerAngles(0,1,2)[1]) );
    current_values.yaw = static_cast<float>( LimitAngle(current_pose.linear().eulerAngles(0,1,2)[2]) );


    current_values_publisher.publish( current_values );

    rate.sleep();
  }

//  emitter << YAML::BeginMap;

//  emitter << YAML::Key << "desired_values";
//  emitter << YAML::Value << YAML::BeginMap;
//      emitter << YAML::Key << "z_desired" << YAML::Value << YAML::Flow << z_des;
//  emitter << YAML::EndMap;
//  emitter << YAML::Newline;
//  emitter << YAML::Value << YAML::BeginMap;
//      emitter << YAML::Key << "roll_desired" << YAML::Value << YAML::Flow << roll_des;
//  emitter << YAML::EndMap;
//  emitter << YAML::Newline;
//  emitter << YAML::Value << YAML::BeginMap;
//      emitter << YAML::Key << "pitch_desired" << YAML::Value << YAML::Flow << pitch_des;
//  emitter << YAML::EndMap;
//  emitter << YAML::Newline;

//  emitter << YAML::Key << "current_values";
//  emitter << YAML::Value << YAML::BeginMap;
//      emitter << YAML::Key << "z" << YAML::Value << YAML::Flow << z;
//  emitter << YAML::EndMap;
//  emitter << YAML::Newline;
//  emitter << YAML::Value << YAML::BeginMap;
//      emitter << YAML::Key << "roll" << YAML::Value << YAML::Flow << roll;
//  emitter << YAML::EndMap;
//  emitter << YAML::Newline;
//  emitter << YAML::Value << YAML::BeginMap;
//      emitter << YAML::Key << "pitch" << YAML::Value << YAML::Flow << pitch;
//  emitter << YAML::EndMap;
//  emitter << YAML::Newline;

//  emitter << YAML::EndMap;


  return 0;
}
