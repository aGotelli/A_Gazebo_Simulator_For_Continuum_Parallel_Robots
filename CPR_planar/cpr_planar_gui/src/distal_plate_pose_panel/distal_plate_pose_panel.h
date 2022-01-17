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



#ifndef DISTAL_PLATE_POSE_PANEL_H
#define DISTAL_PLATE_POSE_PANEL_H

#ifdef ROS
#include "src/controllers/generic_panel.h"

#include <geometry_msgs/Pose2D.h>

#include <ros/ros.h>
#include <gazebo_msgs/GetLinkState.h>
#else
#include "generic_panel.h"


namespace geometry_msgs
{
struct Point{
  Point()=default;
  double x { 0.0 };
  double y { 0.0 };
  double z { 0.0 };
};
struct Quaternion {
  Quaternion()=default;
  double w { 0.0 };
  double x { 0.0 };
  double y { 0.0 };
  double z { 0.0 };
};
struct Pose {
  Pose()=default;
  Point position;
  Quaternion orientation;
};

struct Pose2D {
    Pose2D()=default;
    double x { 0.0 };
    double y { 0.0 };
    double theta { 0.0 };
};
}
#endif

#include <eigen3/Eigen/Dense>


class DistalPlatePosePanel : public GenericPanel
{

public:
    explicit DistalPlatePosePanel(QWidget *parent_widget=nullptr) : GenericPanel(parent_widget){}
    DistalPlatePosePanel(QWidget *parent_widget,
                         const std::string &panel_title,
                         const QRect &panel_dimensions,
                         const std::string &main_windows_object_name="MainWindow");

#ifdef ROS
    DistalPlatePosePanel(QWidget *parent_widget,
                         const std::string &panel_title,
                         const QRect &panel_dimensions,
                         ros::NodeHandle &nh_,
                         const std::string &main_windows_object_name="MainWindow")
      : DistalPlatePosePanel(parent_widget,
                             panel_title,
                             panel_dimensions,
                             main_windows_object_name) {nh = nh_;}
#endif

    void DisableOnlyOverconstrains(const std::vector<bool> &underactuation_vector);

    virtual void Update() override;

    bool GetPoseIfChanged(geometry_msgs::Pose2D &pose);
private:
    //  Controllers for x,y and z positions
    GenericController *x;
    GenericController *y;
    GenericController *z;

    //  Spacer between translations and rotations controllers
    QSpacerItem *translation_rotations_spacer;

    //  Controllers for roll, pitch and yaw angles
    GenericController *roll;
    GenericController *pitch;
    GenericController *yaw;

    //  Some spaces to keep the panel layout nice
    QSpacerItem *horizontalSpacer;
    QSpacerItem *verticalSpacer;

    //  Limit for the position controllers
    double translation_lower_limit { -2.0 };
    double translation_upper_limit { 2.0 };

    //  Limits for the rotations controllers
    double rotation_lower_limit { -M_PI };
    double rotation_upper_limit { M_PI };

    geometry_msgs::Quaternion ToQuaternion(const Eigen::Vector3d &euler_angles);

    geometry_msgs::Pose2D distal_plate_pose;

#ifdef ROS
    ros::NodeHandle nh;
    ros::ServiceClient distal_plate_state_client;
    gazebo_msgs::GetLinkStateRequest distal_plate_state_request;
    gazebo_msgs::GetLinkStateResponse distal_plate_state_response;
#endif
};

#endif // DISTAL_PLATE_POSE_PANEL_H
