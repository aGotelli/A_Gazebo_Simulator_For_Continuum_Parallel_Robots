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



#include "distal_plate_wrench_panel.h"

DistalPlateWrenchPanel::DistalPlateWrenchPanel(QWidget *parent_widget,
                                               const std::string &panel_title,
                                               const QRect &panel_dimensions,
                                               const std::string &main_windows_object_name)
                        : GenericPanel(parent_widget,
                                       panel_title,
                                       panel_dimensions,
                                       main_windows_object_name)
{
    force_group_box = new QGroupBox(panel_group_box);
    force_group_box->setTitle(QApplication::translate("MainWindow", "Forces", nullptr));
    force_group_box->setObjectName(QString::fromUtf8("forces_groupbox"));

    force_layout = new QGridLayout(force_group_box);
    force_layout->setSpacing(6);
    force_layout->setContentsMargins(11, 11, 11, 11);
    force_layout->setObjectName(QString::fromUtf8("forces_groupbox_layout"));

    ControllerOptions fx_options(0, 0);
    fx_options.SetHorizontalLayout(0, -1, 1);
    fx_options.slider_type = SliderType::NONE;
    fx = new GenericController("fx [N]", force_group_box, force_layout, -10, 10, fx_options);
    QObject::connect(fx, &GenericController::ValueChanged, [this](const double &value){
      this->distal_plate_wrench.fx = value;
      this->has_changed = true;
    });

    ControllerOptions fy_options(1, 0);
    fy_options.SetHorizontalLayout(0, -1, 1);
    fy_options.slider_type = SliderType::NONE;
    fy = new GenericController("fy [N]", force_group_box, force_layout, -10, 10, fy_options);
    QObject::connect(fy, &GenericController::ValueChanged, [this](const double &value){
      this->distal_plate_wrench.fy = value;
      this->has_changed = true;
    });


    ControllerOptions fz_options(2, 0);
    fz_options.SetHorizontalLayout(0, -1, 1);
    fz_options.slider_type = SliderType::NONE;
    fz = new GenericController("fz [N]", force_group_box, force_layout, -10, 10, fz_options);
//    QObject::connect(fz, &GenericController::ValueChanged, [this](const double &value){
//      this->distal_plate_wrench.force.z = value;
//      this->has_changed = true;
//    });
    fz->SetDisabled();


    panel_layout->addWidget(force_group_box, 0, 0);



    torque_group_box = new QGroupBox(panel_group_box);
    torque_group_box->setTitle(QApplication::translate("MainWindow", "Forces", nullptr));
    torque_group_box->setObjectName(QString::fromUtf8("forces_groupbox"));

    torque_layout = new QGridLayout(torque_group_box);
    torque_layout->setSpacing(6);
    torque_layout->setContentsMargins(11, 11, 11, 11);
    torque_layout->setObjectName(QString::fromUtf8("forces_groupbox_layout"));

    ControllerOptions mx_options(0, 0);
    mx_options.SetHorizontalLayout(0, -1, 1);
    mx_options.slider_type = SliderType::NONE;
    mx = new GenericController("mx [Nm]", torque_group_box, torque_layout, -10, 10, mx_options);
//    QObject::connect(mx, &GenericController::ValueChanged, [this](const double &value){
//      this->distal_plate_wrench.torque.x = value;
//      this->has_changed = true;
//    });
    mx->SetDisabled();


    ControllerOptions my_options(1, 0);
    my_options.SetHorizontalLayout(0, -1, 1);
    my_options.slider_type = SliderType::NONE;
    my = new GenericController("my [Nm]", torque_group_box, torque_layout, -10, 10, my_options);
//    QObject::connect(my, &GenericController::ValueChanged, [this](const double &value){
//      this->distal_plate_wrench.torque.y = value;
//      this->has_changed = true;
//    });
    my->SetDisabled();

    ControllerOptions mz_options(2, 0);
    mz_options.SetHorizontalLayout(0, -1, 1);
    mz_options.slider_type = SliderType::NONE;
    mz = new GenericController("mz [Nm]", torque_group_box, torque_layout, -10, 10, mz_options);
    QObject::connect(mz, &GenericController::ValueChanged, [this](const double &value){
      this->distal_plate_wrench.mz = value;
      this->has_changed = true;
    });

    panel_layout->addWidget(torque_group_box, 0, 1);

    SetDisabled();
}



bool DistalPlateWrenchPanel::GetWrenchIfChanged(cpr_planar_messages::Wrench2D &wrench)
{
  if(!this->has_changed)
    return false;

  this->has_changed = false;

  wrench = distal_plate_wrench;

  return true;
}
