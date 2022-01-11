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




#ifndef DISTAL_PLATE_WRENCH_PANEL_H
#define DISTAL_PLATE_WRENCH_PANEL_H

#include <QtWidgets/QGroupBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QApplication>

#ifdef ROS
#include "src/controllers/generic_panel.h"

#include "geometry_msgs/Wrench.h"
#else
#include "generic_panel.h"

namespace geometry_msgs {
struct Force {
  Force()=default;
  double x { 0.0 };
  double y { 0.0 };
  double z { 0.0 };
};
struct Torque {
  Torque()=default;
  double x { 0.0 };
  double y { 0.0 };
  double z { 0.0 };
};

struct Wrench {
  Wrench()=default;
  Force force;
  Torque torque;
};
}
#endif


class DistalPlateWrenchPanel : public GenericPanel
{

public:
    explicit DistalPlateWrenchPanel(QWidget *parent_widget=nullptr) : GenericPanel(parent_widget){}
    DistalPlateWrenchPanel(QWidget *parent_widget,
                           const std::string &panel_title,
                           const QRect &panel_dimensions,
                           const std::string &main_windows_object_name="MainWindow");

    bool GetWrenchIfChanged(geometry_msgs::Wrench &wrench);

private:
    QGroupBox *force_group_box { nullptr };
    QGridLayout *force_layout { nullptr };
    GenericController *fx { nullptr };
    GenericController *fy { nullptr };
    GenericController *fz { nullptr };

    QGroupBox *torque_group_box { nullptr };
    QGridLayout *torque_layout { nullptr };
    GenericController *mx { nullptr };
    GenericController *my { nullptr };
    GenericController *mz { nullptr };

    geometry_msgs::Wrench distal_plate_wrench;
};

#endif // DISTAL_PLATE_WRENCH_PANEL_H
