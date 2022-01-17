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



#ifndef DOFCONTROLLER_H
#define DOFCONTROLLER_H

#include "basic_motion.h"

#include <QtWidgets/QPushButton>

#include <boost/function.hpp>
#include <boost/bind.hpp>

#include <array>

class DOFController
{
public:

    DOFController()=default;

    typedef std::array<std::pair<std::string, bool>, 6>::iterator Iterator;
    void Setup(QWidget *Form, QHBoxLayout *boxes_layout, const std::string &box_name,
               std::array<std::pair<std::string, bool>, 6> &distal_plate_motions,
               const unsigned int &start, const std::shared_ptr<unsigned int> &ptr_remaining_constraint_to_remove,
               boost::function<void()> &fun);

    void DisableAllNotChecked();

    void EnableAll();

    void ResetAll();

private:
    QVBoxLayout *verticalLayout { nullptr };
    QGridLayout *gridLayout { nullptr };
    QGroupBox *groupBox { nullptr };

    QSpacerItem *verticalSpacer { nullptr };
    QSpacerItem *horizontalSpacer { nullptr };

    QPushButton *all_pushButton { nullptr };


    std::array<BasicMotion, 3> basic_motions;
};

#endif // DOFCONTROLLER_H
