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



#ifndef JOINTS_CONTROL_PANEL_H
#define JOINTS_CONTROL_PANEL_H

#ifdef ROS
#include "src/controllers/generic_panel.h"

#include <ros/ros.h>
#include <urdf/model.h>

#include <std_msgs/Float32MultiArray.h>

#include "cpr_planar_messages/JointValue.h"
#else
#include "generic_panel.h"

namespace cpr_planar_messages {
struct JointValue{
  int id;
  float value;
};
}
#endif



/*!
 * \brief The JointsControlPanel class
 */
class JointsControlPanel : public GenericPanel
{

public:
    explicit JointsControlPanel(QWidget *parent_widget=nullptr) : GenericPanel(parent_widget){}
    JointsControlPanel(QWidget *parent_widget,
                       const std::string &panel_title,
                       const QRect &panel_dimensions,
                       const std::string &main_windows_object_name="MainWindow");

#ifdef ROS
    JointsControlPanel(QWidget *parent_widget,
                       const std::string &panel_title,
                       const QRect &panel_dimensions,
                       ros::NodeHandle &nh_,
                       const std::string &main_windows_object_name="MainWindow") :
      JointsControlPanel(parent_widget, panel_title, panel_dimensions, main_windows_object_name)
      {
      nh = nh_;
      }

    virtual void Update()override;

    bool GetJointValuesIfChanged(std::vector<float> &joints_values_);
#endif

public slots:
    void CreateJointWidgets(const unsigned int number_of_limbs);

    virtual void ResetPanel()override;

private:
    QScrollArea *joints_values_scroll_area;
    QWidget *scroll_area_contents;
    QGridLayout *scroll_area_layout;

    unsigned int number_of_limbs { 0 };

    bool need_of_update { false };


    std::vector<GenericController*>joint_controllers;

    std::vector<float> joints_values;

    void GetJointLimits(double &upper_limit, double &lower_limit, const unsigned int index);

    void SetValue(const unsigned int i, const double &value);

    [[nodiscard]]std::string GetJointName(const unsigned int index);

#ifdef ROS
    [[nodiscard]] urdf::JointConstSharedPtr GetJointPointer(const unsigned int index);
#endif

#ifdef ROS
    ros::NodeHandle nh;

    ros::Subscriber joint_values_listener;

    void JointValuesCallBack(const std_msgs::Float32MultiArray::ConstPtr &joint_values_message);
#endif


};

#endif // JOINTS_CONTROL_PANEL_H
