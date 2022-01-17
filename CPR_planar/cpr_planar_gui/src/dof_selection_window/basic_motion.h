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



#ifndef BASIC_MOTION_H
#define BASIC_MOTION_H

#include <QObject>

#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QApplication>


#include <iostream>
#include <memory>
#include <array>


struct DistalPlateMotions {
    DistalPlateMotions()=default;

    std::shared_ptr<unsigned int> remaining_constraint_to_remove{ std::make_shared<unsigned int>(0) };


    mutable std::array<std::pair<std::string, bool>, 6> constrained_motions { std::pair<std::string, bool>{"x", true},
                                                                                std::pair<std::string, bool>{"y", true},
                                                                                std::pair<std::string, bool>{"z", true},
                                                                                std::pair<std::string, bool>{"roll", true},
                                                                                std::pair<std::string, bool>{"pitch", true},
                                                                                std::pair<std::string, bool>{"yaw", true}};
};

class BasicMotion : public QObject
{
    Q_OBJECT
public:
    BasicMotion()=default;

    void SetUp(const std::string &motion_name,
                bool &targeted_distal_plate_dof,
                const std::shared_ptr<unsigned int> &ptr_remaining_constraint_to_remove,
                QVBoxLayout *vertical_layout,
                QGroupBox *groupBox);

    void Reset();

    void DisableIfNotChecked();

    void Enable();

    void SetChecked();

signals:
    void ConstrainChanged();

private:
  QCheckBox *checkBox_ { nullptr };


  /*std::shared_ptr<bool>*/bool *ptr_targeted_motion_is_constrained_ { nullptr };

  std::shared_ptr<unsigned int> ptr_remaining_constraint_to_remove_ { nullptr };



};

#endif // BASIC_MOTION_H
