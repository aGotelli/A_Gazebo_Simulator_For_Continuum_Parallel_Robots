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



#include "generic_panel.h"

GenericPanel::GenericPanel(QWidget *parent_widget,
                           const std::string &panel_title,
                           const QRect &panel_dimensions,
                           const std::string &main_windows_object_name)
    : QWidget(parent_widget)
{
    //  Setting up the object name
    this->setObjectName( QString::fromStdString(panel_title) );

    //  Setting up the group box
    panel_group_box = new QGroupBox(parent_widget);
    panel_group_box->setObjectName( QString::fromStdString(panel_title+"_group_box") );
    panel_group_box->setGeometry(panel_dimensions);
    panel_group_box->setTitle(QApplication::translate(main_windows_object_name.data(), panel_title.data(), nullptr));

    //  Setting up the main layout
    panel_layout = new QGridLayout(panel_group_box);
    panel_layout->setSpacing(6);
    panel_layout->setContentsMargins(11, 11, 11, 11);
    panel_layout->setObjectName(QString::fromStdString(panel_title+"_group_box_layout"));
}

void GenericPanel::ErrorLogThisPanel(const std::string & error_message)
{
    std::cout << "For " << this->objectName().toStdString() << " ";
    std::cout << error_message << std::endl;
}
