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



#include "distal_plate_pose_panel.h"

DistalPlatePosePanel::DistalPlatePosePanel(QWidget *parent_widget,
                                           const std::string &panel_title,
                                           const QRect &panel_dimensions,
                                           const std::string &main_windows_object_name)
                        : GenericPanel(parent_widget,
                                       panel_title,
                                       panel_dimensions,
                                       main_windows_object_name)
{
    ControllerOptions x_options(0, 0);
    x_options.SetHorizontalLayout(0, 1, 2);
    x = new GenericController("x", panel_group_box, panel_layout,
                              translation_lower_limit,
                              translation_upper_limit,
                              x_options);
    QObject::connect(x, &GenericController::ValueChanged, [this](const double &value){
      this->distal_plate_pose.x = value;
      this->has_changed = true;
    });

    ControllerOptions y_options(1, 0);
    y_options.SetHorizontalLayout(0, 1, 2);
    y = new GenericController("y", panel_group_box, panel_layout,
                              translation_lower_limit,
                              translation_upper_limit,
                              y_options);
    QObject::connect(y, &GenericController::ValueChanged, [this](const double &value){
      this->distal_plate_pose.y = value;
      this->has_changed = true;
    });

    ControllerOptions z_options(2, 0);
    z_options.SetHorizontalLayout(0, 1, 2);
    z = new GenericController("z", panel_group_box, panel_layout,
                              translation_lower_limit,
                              translation_upper_limit,
                              z_options);
//    QObject::connect(z, &GenericController::ValueChanged, [this](const double &value){
//      this->distal_plate_pose.position.z = value;
//      this->has_changed = true;
//    });
    z->SetDisabled();

    translation_rotations_spacer = new QSpacerItem(10, 10, QSizePolicy::Expanding, QSizePolicy::Minimum);
    panel_layout->addItem(translation_rotations_spacer, 0, 1, 3);


    ControllerOptions roll_options(0, 2);
    roll_options.SetVerticalLayout(0, 2, 1);
    roll_options.SetSpan(3, 1);
    roll_options.slider_type = SliderType::DIAL;
    roll_options.slider_min_size = QSize(50, 50);
    roll_options.slider_max_size = QSize(75, 75);
    roll = new GenericController("roll", panel_group_box, panel_layout,
                                 rotation_lower_limit, rotation_upper_limit, roll_options);
//    QObject::connect(roll, &GenericController::ValueChanged, [this](const double &value){
//      Eigen::Vector3d euler_angles;
//      euler_angles << value, this->pitch->GetValue(), this->yaw->GetValue();

//      this->distal_plate_pose.orientation = this->ToQuaternion(euler_angles);
//      this->has_changed = true;
//    });
    roll->SetDisabled();

    ControllerOptions pitch_options(0, 3);
    pitch_options.SetVerticalLayout(0, 2, 1);
    pitch_options.SetSpan(3, 1);
    pitch_options.slider_type = SliderType::DIAL;
    pitch_options.slider_min_size = QSize(50, 50);
    pitch_options.slider_max_size = QSize(75, 75);
    pitch = new GenericController("pitch", panel_group_box, panel_layout,
                                  rotation_lower_limit, rotation_upper_limit, pitch_options);
//    QObject::connect(pitch, &GenericController::ValueChanged, [this](const double &value){
//      Eigen::Vector3d euler_angles;
//      euler_angles << this->roll->GetValue(), value, this->yaw->GetValue();

//      this->distal_plate_pose.orientation = this->ToQuaternion(euler_angles);
//      this->has_changed = true;
//    });
    pitch->SetDisabled();

    ControllerOptions yaw_options(0, 4);
    yaw_options.SetVerticalLayout(0, 2, 1);
    yaw_options.SetSpan(3, 1);
    yaw_options.slider_type = SliderType::DIAL;
    yaw_options.slider_min_size = QSize(50, 50);
    yaw_options.slider_max_size = QSize(75, 75);
    yaw = new GenericController("yaw", panel_group_box, panel_layout,
                                rotation_lower_limit, rotation_upper_limit, yaw_options);
    QObject::connect(yaw, &GenericController::ValueChanged, [this](const double &value){
//      Eigen::Vector3d euler_angles;
//      euler_angles << this->roll->GetValue(), this->pitch->GetValue(), value;

//      this->distal_plate_pose.orientation = this->ToQuaternion(euler_angles);
      this->distal_plate_pose.theta = value;
      this->has_changed = true;
    });

#ifdef ROS
    distal_plate_state_client = nh.serviceClient<gazebo_msgs::GetLinkState>("/gazebo/get_link_state");
    distal_plate_state_request.link_name = "distal_plate::distal_plate";
    distal_plate_state_request.reference_frame = "world";
#endif

    SetDisabled();
}

void DistalPlatePosePanel::DisableOnlyOverconstrains(const std::vector<bool> &underactuation_vector)
{
    if(!underactuation_vector[0])
        x->SetDisabled();

    if(!underactuation_vector[1])
        y->SetDisabled();

//    if(!underactuation_vector[2])
//        z->SetDisabled();

//    if(!underactuation_vector[3])
//        roll->SetDisabled();

//    if(!underactuation_vector[4])
//        pitch->SetDisabled();

//    if(!underactuation_vector[5])
//        yaw->SetDisabled();
    if(!underactuation_vector[2])
        yaw->SetDisabled();
}


void DistalPlatePosePanel::Update()
{
#ifdef ROS
  distal_plate_state_client.call(distal_plate_state_request, distal_plate_state_response);

  if(distal_plate_state_response.success == false) {
    ErrorLogThisPanel("is trying to update the distal plate pose inconsistenly");
    return;
  }

//  distal_plate_pose = distal_plate_state_response.link_state.pose;
  distal_plate_pose.x = distal_plate_state_response.link_state.pose.position.x;
  distal_plate_pose.y = distal_plate_state_response.link_state.pose.position.y;

  distal_plate_pose.theta = distal_plate_state_response.link_state.pose.position.x;


  x->SetValue(distal_plate_pose.x);
  y->SetValue(distal_plate_pose.y);
//  z->SetValue(distal_plate_pose.position.z);


//  Eigen::Quaternion<double> q(distal_plate_pose.orientation.w,
//                              distal_plate_pose.orientation.x,
//                              distal_plate_pose.orientation.y,
//                              distal_plate_pose.orientation.z);

//  Eigen::Matrix3d R = q.toRotationMatrix();

//  Eigen::Vector3d euler = R.eulerAngles(0, 1, 2);

//  if(R.isIdentity())
//   euler = Eigen::Vector3d(0.0, 0.0, 0.0);

//  Eigen::Matrix3d R_obtained;
//  R_obtained = Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX())
//              * Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY())
//              * Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ());

  Eigen::Quaternion<double> q(distal_plate_state_response.link_state.pose.orientation.w,
                              distal_plate_state_response.link_state.pose.orientation.x,
                              distal_plate_state_response.link_state.pose.orientation.y,
                              distal_plate_state_response.link_state.pose.orientation.z);

  distal_plate_pose.theta = Eigen::Rotation2D<double>( q.toRotationMatrix().block<2,2>(0,0) ).angle();


//  roll->SetValue(euler[0]);
//  pitch->SetValue(euler[1]);
  yaw->SetValue(distal_plate_pose.theta);
#endif
}


bool DistalPlatePosePanel::GetPoseIfChanged(geometry_msgs::Pose2D &pose)
{
  if(!this->has_changed)
    return false;

  this->has_changed = false;

  pose = distal_plate_pose;

  return true;
}

geometry_msgs::Quaternion DistalPlatePosePanel::ToQuaternion(const Eigen::Vector3d &euler_angles)
{
  Eigen::Quaternion<double> q;
  q = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitX())
      * Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY())
      * Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitZ());

  geometry_msgs::Quaternion quaternion;
  quaternion.w =q.w();
  quaternion.x =q.x();
  quaternion.y =q.y();
  quaternion.z =q.z();

  return quaternion;
}
