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
  #include "src/dof_selection_window/basic_motion.h"
#else
  #include "basic_motion.h"
#endif

void BasicMotion::SetUp(const std::string &motion_name,
                         bool &targeted_motion_is_constrained,
                         const std::shared_ptr<unsigned int> &ptr_remaining_constraint_to_remove,
                         QVBoxLayout *vertical_layout,
                         QGroupBox *groupBox)
//                        : targeted_distal_plate_dof_( std::make_shared<bool>(targeted_distal_plate_dof)  ),
//                          number_of_distal_plate_dofs_( std::make_shared<unsigned int>(number_of_distal_plate_dofs) )


{
    ptr_targeted_motion_is_constrained_ = /*std::make_shared<bool>(targeted_distal_plate_dof)*/&targeted_motion_is_constrained;

    ptr_remaining_constraint_to_remove_ = ptr_remaining_constraint_to_remove;


    //  The initialize the actual Qt object
    checkBox_ = new QCheckBox(groupBox);
    checkBox_->setObjectName( QString::fromStdString(motion_name+"_checkBox") );
    checkBox_->setText(QApplication::translate("Form", motion_name.data(), nullptr));

    vertical_layout->addWidget(checkBox_);

    QObject::connect(checkBox_, &QCheckBox::clicked, [this](){
        if(this->checkBox_->isChecked()) {
            *this->ptr_targeted_motion_is_constrained_ = false;
            *this->ptr_remaining_constraint_to_remove_ -= 1;
        } else {
            *this->ptr_targeted_motion_is_constrained_ = true;
            *this->ptr_remaining_constraint_to_remove_ += 1;
        }
        emit this->ConstrainChanged();
    });


}


void BasicMotion::Reset()
{
  if(checkBox_->isChecked()) {
      checkBox_->setChecked(false);
      *ptr_remaining_constraint_to_remove_ += 1;
      *ptr_targeted_motion_is_constrained_ = true;
  }
}

void BasicMotion::SetChecked()
{
    //  If not checked, then add remove the degree of freedom
    if(!checkBox_->isChecked()) {
        checkBox_->setChecked(true);
        *ptr_targeted_motion_is_constrained_ = false;
        *ptr_remaining_constraint_to_remove_ -= 1;
        emit ConstrainChanged();
    }

    checkBox_->setChecked(true);
}

void BasicMotion::Enable()
{
    checkBox_->setEnabled(true);
}


void BasicMotion::DisableIfNotChecked()
{
  if(!checkBox_->isChecked())
      checkBox_->setDisabled(true);
}



