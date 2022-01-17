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
  #include "src/dof_selection_window/dof_selection_window.h"
#else
  #include "dof_selection_window.h"
#endif




//#ifdef ROS
//#include "robot_gui/dof_selection_widget.h"
//#else
//#include "include/robot_gui/dof_selection_widget.h"
//#endif

//DOFWidget::DOFWidget(QWidget *parent) : QWidget(parent)
//{

//}



//SecondWindow::SecondWindow(QWidget *parent):
//    QWidget(parent)
DofSelectionWindow::DofSelectionWindow(const unsigned int number_of_dof_)
    //: distal_plate_dofs(number_of_dof_)
{

    setObjectName(QString::fromUtf8("Distal Plate DOF Selection"));
    setWindowTitle(QApplication::translate("Distal Plate DOF Selection", "Select The Distal Plate DOFs", nullptr));
    setWindowState(Qt::WindowState::WindowActive);
    setWindowFlags(Qt::WindowStaysOnTopHint);
    resize(400, 190);
    window_gridLayout = new QGridLayout(this);
    window_gridLayout->setObjectName(QString::fromUtf8("window_gridLayout"));
    group_boxes_horizontalLayout = new QHBoxLayout();
    group_boxes_horizontalLayout->setObjectName(QString::fromUtf8("group_boxes_horizontalLayout"));

    //  Set the number of deegree of freedom to obtain
    *distal_plate_motions.remaining_constraint_to_remove = number_of_dof_;

    boost::function<void()> fun = boost::bind(&DofSelectionWindow::DOFAdded, this);


    translations_dofs_controller.Setup(this, group_boxes_horizontalLayout, "translations",
                                       distal_plate_motions.constrained_motions, 0,
                                       distal_plate_motions.remaining_constraint_to_remove, fun);

    rotations_dofs_controller.Setup(this, group_boxes_horizontalLayout, "rotations",
                                    distal_plate_motions.constrained_motions, 3,
                                    distal_plate_motions.remaining_constraint_to_remove, fun);


    window_gridLayout->addLayout(group_boxes_horizontalLayout, 0, 0, 1, 1);

    buttonBox = new QDialogButtonBox(this);
    buttonBox->setObjectName(QString::fromUtf8("buttonBox"));
    buttonBox->setStandardButtons(QDialogButtonBox::Ok|QDialogButtonBox::Reset);

    buttonBox->buttons()[0]->setDisabled(true);

    window_gridLayout->addWidget(buttonBox, 1, 0, 1, 1);

    QMetaObject::connectSlotsByName(this);


    QObject::connect(buttonBox->buttons()[1], &QPushButton::clicked, [this](){
        this->translations_dofs_controller.ResetAll();
        this->translations_dofs_controller.ResetAll();
        this->buttonBox->buttons()[0]->setDisabled(true);
        translations_dofs_controller.EnableAll();
        rotations_dofs_controller.EnableAll();
    });

    QObject::connect(buttonBox->buttons()[0], &QPushButton::clicked, [this](){
        this->buttonBox->setDisabled(true);
        emit this->DistalPlateDOFsDecided( ToStdVector(this->distal_plate_motions) );
        this->finished = true;
        this->close();
    });

}

std::vector<bool> DofSelectionWindow::ToStdVector(const DistalPlateMotions) const
{
    std::vector<bool> vector(3);
    for(unsigned int i=0; i<3; i++)
        vector[i] = this->distal_plate_motions.constrained_motions[i].second;

    return vector;
}

void DofSelectionWindow::closeEvent (QCloseEvent *event)
{
    if(finished){
        event->accept();
        return;
    }

    if(*distal_plate_motions.remaining_constraint_to_remove == static_cast<unsigned int>(0)) {
        QMessageBox::StandardButton response = QMessageBox::question(this, QString::fromStdString(""),
                                                                    QString::fromStdString("Do you want to use these Degrees of Freedom?"));
        if(response == QMessageBox::Yes){
            emit this->DistalPlateDOFsDecided( ToStdVector(this->distal_plate_motions) );
            event->accept();
        }
        if(response == QMessageBox::No)
            event->ignore();
        return;
    }


    const std::string information_message = "You still have to select " + std::to_string(*distal_plate_motions.remaining_constraint_to_remove) + "DOFs";
    QMessageBox::information(this, QString::fromStdString("Overconstrained Problem"), QString::fromStdString(information_message));

    event->ignore();

}


