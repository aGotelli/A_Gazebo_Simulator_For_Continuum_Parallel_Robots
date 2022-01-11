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




#include "joints_control_panel.h"

JointsControlPanel::JointsControlPanel(QWidget *parent_widget,
                                       const std::string &panel_title,
                                       const QRect &panel_dimensions,
                                       const std::string &main_windows_object_name)
                : GenericPanel(parent_widget,
                               panel_title,
                               panel_dimensions,
                               main_windows_object_name)
{
    joints_values_scroll_area = new QScrollArea(panel_group_box);
    joints_values_scroll_area->setObjectName(QString::fromUtf8("joints_values_scrollArea"));
    joints_values_scroll_area->setWidgetResizable(true);
    scroll_area_contents = new QWidget();
    scroll_area_contents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
    scroll_area_contents->setGeometry(QRect(0, 0, 235, 145));
    joints_values_scroll_area->setWidget(scroll_area_contents);

    panel_layout->addWidget( joints_values_scroll_area );

#ifdef ROS
    joint_values_listener = nh.subscribe("/joint_values", 1, &JointsControlPanel::JointValuesCallBack, this);
#endif

    SetDisabled();
}

void JointsControlPanel::CreateJointWidgets(const unsigned int number_of_limbs_)
{
    number_of_limbs = number_of_limbs_;
    scroll_area_contents = new QWidget();
    scroll_area_contents->setObjectName(QString::fromUtf8("scrollAreaWidgetContents"));
    scroll_area_contents->setGeometry(QRect(0, 0, 230, 250));
    joints_values_scroll_area->setWidget(scroll_area_contents);

    scroll_area_layout = new QGridLayout(scroll_area_contents);
    scroll_area_layout->setSpacing(10);
    scroll_area_layout->setContentsMargins(11, 11, 11, 11);
    scroll_area_layout->setObjectName(QString::fromUtf8("scroll_area_layout"));


    joint_controllers.resize( number_of_limbs );

    joints_values.resize( number_of_limbs );

    double upper_limit;
    double lower_limit;
    for(unsigned int i=0;i<number_of_limbs;i++){

    GetJointLimits(upper_limit, lower_limit, i);

    const std::string joint_name = GetJointName(i);

    ControllerOptions joint_options(static_cast<int>( i ), 0);
    joint_options.SetHorizontalLayout(0, 1, 2);
    joint_controllers[i] = new GenericController(joint_name,
                                                 scroll_area_contents,
                                                 scroll_area_layout,
                                                 lower_limit,
                                                 upper_limit,
                                                 joint_options);
    QObject::connect(joint_controllers[i], &GenericController::ValueChanged, [this, i](const double &value){
      this->SetValue(i, value);
      this->has_changed = true;
    });


    }
    joints_values_scroll_area->setWidget(scroll_area_contents);
    panel_layout->addWidget(joints_values_scroll_area, 1, 0, 1, 1);
}

void JointsControlPanel::SetValue(const unsigned int i, const double &value)
{
  joints_values[i] = static_cast<float>(value);
}


void JointsControlPanel::GetJointLimits(double &upper_limit,
                                        double &lower_limit,
                                        [[maybe_unused]]const unsigned int index)
{
#ifndef ROS
    upper_limit = 1.0;
    lower_limit = -1.0;
#else
    //joints_values.joints[i].id = i;

    const auto base_joint_ptr = GetJointPointer(index);

    upper_limit = 0.0;
    lower_limit = 0.0;

    if(base_joint_ptr->type == urdf::Joint::CONTINUOUS) {
      upper_limit = M_PI;
      lower_limit = -M_PI;
    }

    if(base_joint_ptr->type == urdf::Joint::PRISMATIC) {
      upper_limit = base_joint_ptr->limits->upper;
      lower_limit = base_joint_ptr->limits->lower;
    }

    if(base_joint_ptr->type == urdf::Joint::REVOLUTE) {
      upper_limit = base_joint_ptr->limits->upper;
      lower_limit = base_joint_ptr->limits->lower;
    }

    if(base_joint_ptr->type == urdf::Joint::FIXED) {
      upper_limit = 3.0;
      lower_limit = 0.0;
    }
#endif
}

std::string JointsControlPanel::GetJointName(const unsigned int index)
{
#ifndef ROS
    return "joint" + std::to_string(index+1);
#else
    const auto joint_pointer = GetJointPointer(index);

    return joint_pointer->name + std::to_string(index);
#endif
}

void JointsControlPanel::ResetPanel()
{
  delete joints_values_scroll_area->takeWidget();
}

#ifdef ROS
urdf::JointConstSharedPtr JointsControlPanel::GetJointPointer(const unsigned int index)
{
  const std::string limb_prefix = [&](){
    std::string limb_prefix_;
    nh.param<std::string>("limb_prefix", limb_prefix_, "limb");
    return limb_prefix_;
  }();

  const std::string limb_name = limb_prefix + std::to_string(index+1);

  std::string limb_description;
  nh.getParam("/" + limb_name + "/robot_description", limb_description);

  urdf::Model limb_model;

  if(!limb_model.initString( limb_description ))
    ROS_ERROR_STREAM("The Given robot model does not exist");

  return limb_model.getJoint("base_joint");
}


void JointsControlPanel::Update()
{
  need_of_update = true;
}

bool JointsControlPanel::GetJointValuesIfChanged(std::vector<float> &joints_values_)
{
  if(!this->has_changed)
    return false;

  this->has_changed = false;

  joints_values_ = joints_values;

  return true;
}

void JointsControlPanel::JointValuesCallBack(const std_msgs::Float32MultiArray::ConstPtr &joint_values_message)
{
  if(!need_of_update)
    return;

  //  Else update
  if(joints_values.size() != joint_values_message->data.size()) {
    ErrorLogThisPanel("Attempting to copy joints values in an vector of different size");
    return;
  }

  if(joints_values.empty() || joint_controllers.empty()
     || joints_values.size() != joint_controllers.size()) {
    ErrorLogThisPanel("is trying to update inconsistently the joints values");
    return;
  }

  for(unsigned int i=0; i<joint_values_message->data.size(); i++) {
    joints_values[i] = joint_values_message->data[i];
    joint_controllers[i]->SetValue( static_cast<double>(joints_values[i]) );
  }

  //Set the Flag Back
  need_of_update = false;

}


#endif
