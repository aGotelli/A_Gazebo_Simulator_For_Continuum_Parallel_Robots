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



#ifndef DOF_SELECTION_WINDOW_H
#define DOF_SELECTION_WINDOW_H

#include <QWidget>

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QWidget>
#include <QCloseEvent>
#include <QMessageBox>

#ifdef ROS
  #include "src/dof_selection_window/dof_controller.h"
#else
  #include "dof_controller.h"
#endif


class DofSelectionWindow : public QWidget
{
    Q_OBJECT
public:
    DofSelectionWindow()=default;

    DofSelectionWindow(const unsigned int constraints_to_remove_);
    void closeEvent (QCloseEvent *event) override;


  signals:

    void DistalPlateDOFsDecided(const std::vector<bool> &underactuation_vector);


  private:
    bool finished { false };

    mutable DistalPlateMotions distal_plate_motions;

    QGridLayout *window_gridLayout { nullptr };
    QHBoxLayout *group_boxes_horizontalLayout { nullptr };
    QDialogButtonBox *buttonBox { nullptr };

  //  dof_group_box translations_group_box;
  //  dof_group_box rotations_group_box;

    DOFController translations_dofs_controller;
    DOFController rotations_dofs_controller;


    void DOFAdded()
    {
        std::cout << "number_of_distal_plate_dofs : " << *distal_plate_motions.remaining_constraint_to_remove << std::endl;
        if(*distal_plate_motions.remaining_constraint_to_remove == 0) {
          translations_dofs_controller.DisableAllNotChecked();
          rotations_dofs_controller.DisableAllNotChecked();

          buttonBox->buttons()[0]->setEnabled(true);
        }

        if(*distal_plate_motions.remaining_constraint_to_remove != 0) {
          translations_dofs_controller.EnableAll();
          rotations_dofs_controller.EnableAll();
        }

    }

    std::vector<bool> ToStdVector(const DistalPlateMotions) const;
};

#endif // DOF_SELECTION_WINDOW_H
