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



#ifndef GENERIC_CONTROLLER_H
#define GENERIC_CONTROLLER_H

#include <QObject>

#include <QtWidgets/QDial>
#include <QtWidgets/QSlider>
#include <QtWidgets/QLabel>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QAbstractSlider>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QApplication>

#include <iostream>
#include <math.h>

enum SliderType{
    HORIZONTAL_SLIDER,
    VERTICAL_SLIDER,
    DIAL,
    NONE
};

struct ControllerOptions {
    ControllerOptions()=default;

    ControllerOptions(const int main_row_pos_,
                      const int main_col_pos_)
                      : main_row_pos(main_row_pos_),
                        main_col_pos(main_col_pos_)   {}

    void SetVerticalLayout(const int label_row_pos_=0,
                           const int slider_row_pos_=1,
                           const int spin_box_row_pos_=2                                   )
    {
        label_row_pos = label_row_pos_;
        slider_row_pos = slider_row_pos_;
        spin_box_row_pos = spin_box_row_pos_;

        label_col_pos = 0;
        slider_col_pos = 0;
        spin_box_col_pos = 0;
    }

    void SetHorizontalLayout(const int label_col_pos_=0,
                             const int slider_col_pos_=1,
                             const int spin_box_col_pos_=2                                   )
    {
        label_col_pos = label_col_pos_;
        slider_col_pos = slider_col_pos_;
        spin_box_col_pos = spin_box_col_pos_;

        label_row_pos = 0;
        slider_row_pos = 0;
        spin_box_row_pos = 0;
    }

    void SetSpan(const int row_span, const int col_span)
    {
        main_row_span = row_span;
        main_col_span = col_span;
    }


    int main_row_pos { 0 };
    int main_col_pos { 0 };

    int main_row_span { 1 };
    int main_col_span { 1 };

    int label_row_pos { 0 };
    int label_col_pos { 0 };

    int slider_row_pos { 0 };
    int slider_col_pos { 0 };

    int spin_box_row_pos { 0 };
    int spin_box_col_pos { 0 };

    QSize slider_min_size { QSize(125, 25) };
    QSize slider_max_size { QSize(75, 25) };

    double spin_box_step { 0.005 };
    int spin_box_decimals { 3 };



    SliderType slider_type { SliderType::HORIZONTAL_SLIDER };

};


class GenericController : public QObject
{
    Q_OBJECT
public:
    GenericController()=default;

    GenericController(const double &lower_limit_,
                       const double &upper_limit_)
        : lower_limit(lower_limit_),
          range(upper_limit_ - lower_limit_){}

    GenericController(const std::string &controller_name,
                       QWidget *parent_widget,
                       QGridLayout *parent_layout,
                       const double &lower_limit_,
                       const double &upper_limit_,
                       const ControllerOptions &controller_options);

    inline double GetValue(){return spin_box->value();}


public slots:
    virtual void SetValue(const double &value);

    void SetDisabled();

    void SetEnabled();

protected slots:
    virtual void FromSliderToSpinBox(const double &slider_value);

    virtual void FromSpinBoxToSlider(const double &spin_box_value);

signals:
    void ValueChanged(const double &value);

protected:
    QLabel *label { nullptr };
    QDoubleSpinBox *spin_box { nullptr };
    QAbstractSlider *slider { nullptr };
    QGridLayout *controller_grid_layout { nullptr };

    const int slider_range { 10000 };


    void ConnectInputs();

private:

    void ErrorLogThisObject(const std::string &error_message);

    double lower_limit { 0.0 };
    double upper_limit { 0.0 };

    double range { -1.0 };



};

#endif // GENERIC_CONTROLLER_H
