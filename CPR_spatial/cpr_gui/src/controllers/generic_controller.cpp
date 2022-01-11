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




#include "generic_controller.h"


GenericController::GenericController(const std::string &angle_name,
                                       QWidget *parent_widget,
                                       QGridLayout *parent_layout,
                                       const double &lower_limit_,
                                       const double &upper_limit_,
                                       const ControllerOptions &controller_options)
                                        : lower_limit(lower_limit_),
                                          upper_limit(upper_limit_),
                                          range(upper_limit - lower_limit)
{
    setObjectName( QString::fromStdString("distal_plate_"+angle_name+"_controller") );

    controller_grid_layout = new QGridLayout();
    controller_grid_layout->setSpacing(6);
    controller_grid_layout->setObjectName( QString::fromStdString(angle_name+"_grid_layout") );

    label = new QLabel(parent_widget);
    label->setObjectName( QString::fromStdString(angle_name+"_label") );
    label->setText(QApplication::translate("MainWindow", angle_name.data(), nullptr));
    controller_grid_layout->addWidget(label, controller_options.label_row_pos, controller_options.label_col_pos, 1, 1);

    if(controller_options.slider_type != SliderType::NONE){
        if(controller_options.slider_type == SliderType::HORIZONTAL_SLIDER){
            slider = new QSlider(parent_widget);
            slider->setOrientation(Qt::Horizontal);
        }

        if(controller_options.slider_type == SliderType::VERTICAL_SLIDER){
            slider = new QSlider(parent_widget);
            slider->setOrientation(Qt::Vertical);
        }

        if(controller_options.slider_type == SliderType::DIAL)
            slider = new QDial(parent_widget);

        slider->setObjectName( QString::fromStdString(angle_name+"_dial") );
        slider->setRange(0, static_cast<int>(slider_range));
        slider->setValue(static_cast<int>(slider_range/2));
        slider->setMinimumSize( controller_options.slider_min_size );
        slider->setMaximumSize( controller_options.slider_max_size );
        controller_grid_layout->addWidget(slider, controller_options.slider_row_pos, controller_options.slider_col_pos, 1, 1);
    }


    spin_box = new QDoubleSpinBox(parent_widget);
    spin_box->setObjectName( QString::fromStdString(angle_name+"_spin_box") );
    spin_box->setMinimumSize(QSize(50, 0));
    spin_box->setMaximumSize(QSize(75, 25));
    spin_box->setRange(lower_limit, upper_limit);
    spin_box->setSingleStep(controller_options.spin_box_step);
    spin_box->setDecimals(controller_options.spin_box_decimals);
    controller_grid_layout->addWidget(spin_box, controller_options.spin_box_row_pos, controller_options.spin_box_col_pos, 1, 1);


    parent_layout->addLayout(controller_grid_layout, controller_options.main_row_pos,
                                                     controller_options.main_col_pos,
                                                     controller_options.main_row_span,
                                                     controller_options.main_col_span);
    if(controller_options.slider_type == SliderType::NONE)
        QObject::connect(spin_box,
                         static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged),
                         this,
                         &GenericController::SetValue);
    else
        ConnectInputs();
}




void GenericController::SetValue(const double &value)
{
    if(spin_box == nullptr) {
        ErrorLogThisObject("is trying to change values to null pointers");
        return;
    }

    if(value < lower_limit || value > upper_limit)
      ErrorLogThisObject("the given value is outside the boudaries");

    spin_box->setValue(value);

    emit ValueChanged(value);
}


void GenericController::SetDisabled()
{
    label->setDisabled(true);
    slider->setDisabled(true);
    spin_box->setDisabled(true);
}

void GenericController::SetEnabled()
{
    label->setEnabled(true);
    slider->setEnabled(true);
    spin_box->setEnabled(true);
}


void GenericController::FromSliderToSpinBox(const double &slider_value)
{
    if(range < 0.0) {
        ErrorLogThisObject("has not a proper scale");
        return;
    }
//    const double spin_box_value = scale*(slider_value - slider_range/2)/(slider_range/2);
    const double spin_box_value = (slider_value / slider_range)*range + lower_limit;

    SetValue(spin_box_value);
}

void GenericController::FromSpinBoxToSlider(const double &spin_box_value)
{
    if(range < 0.0) {
        ErrorLogThisObject("has not a proper scale");
        return;
    }

    //  Given the angle, compute how the slider position changes
//    double slider_value = spin_box_value*(slider_range/2)/scale + (slider_range/2);
    double slider_value = slider_range*(spin_box_value - lower_limit)/range ;


    if(slider == nullptr)
        return;

    //  Apply the value
    slider->setValue(static_cast<int>(slider_value));

}

void GenericController::ErrorLogThisObject(const std::string & error_message)
{
    std::cout << "For " << this->objectName().toStdString() << " ";
    std::cout << error_message << std::endl;
}

void GenericController::ConnectInputs()
{
    QObject::connect(slider, &QAbstractSlider::valueChanged, this, &GenericController::FromSliderToSpinBox);

    QObject::connect(spin_box, static_cast<void (QDoubleSpinBox::*)(double)>(&QDoubleSpinBox::valueChanged), this, &GenericController::FromSpinBoxToSlider);

}
