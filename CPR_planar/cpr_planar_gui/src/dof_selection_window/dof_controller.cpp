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



#ifdef ROS
  #include "src/dof_selection_window/dof_controller.h"
#else
  #include "dof_controller.h"
#endif

void DOFController::Setup(QWidget *Form, QHBoxLayout *boxes_layout, const std::string &box_name,
                          std::array<std::pair<std::string, bool>, 6> &distal_plate_motions,
                          const unsigned int &start, const std::shared_ptr<unsigned int> &ptr_remaining_constraint_to_remove,
                          boost::function<void()> &fun)
{
    groupBox = new QGroupBox(Form);
    groupBox->setObjectName( QString::fromStdString(box_name+"_groupBox") );
    groupBox->setTitle(QApplication::translate("Form", "Distal Plate Translations", nullptr));

    gridLayout = new QGridLayout(groupBox);
    gridLayout->setObjectName( QString::fromStdString(box_name+"_gridLayout") );

    verticalLayout = new QVBoxLayout();
    verticalLayout->setObjectName(QString::fromStdString(box_name+"_xyz_verticalLayout") );

    for (unsigned int i=0;i<3;i++) {

        basic_motions[i].SetUp(distal_plate_motions[start + i].first,
                                distal_plate_motions[start + i].second,
                                ptr_remaining_constraint_to_remove, verticalLayout, groupBox);

    }


    gridLayout->addLayout(verticalLayout, 0, 0, 2, 1);

    verticalSpacer = new QSpacerItem(20, 65, QSizePolicy::Minimum, QSizePolicy::Expanding);

    gridLayout->addItem(verticalSpacer, 0, 2, 1, 1);

    horizontalSpacer = new QSpacerItem(75, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

    gridLayout->addItem(horizontalSpacer, 1, 1, 1, 1);

    all_pushButton = new QPushButton(groupBox);
    all_pushButton->setObjectName(QString::fromStdString(box_name+"_all_pushButton"));
    all_pushButton->setText(QApplication::translate("Form", "All", nullptr));

    gridLayout->addWidget(all_pushButton, 1, 2, 1, 1);


    boxes_layout->addWidget(groupBox);

    QObject::connect(all_pushButton, &QCheckBox::clicked, [this](){
        for(auto& motion : this->basic_motions)
            motion.SetChecked();
    });

    for(auto& motion : basic_motions)
        QObject::connect(&motion, &BasicMotion::ConstrainChanged, Form, fun);

}

void DOFController::DisableAllNotChecked()
{
    for(auto& motion : basic_motions)
        motion.DisableIfNotChecked();
}

void DOFController::EnableAll()
{
    for(auto& motion : basic_motions)
        motion.Enable();
}

void DOFController::ResetAll()
{
    for(auto& motion : basic_motions)
        motion.Reset();
}
