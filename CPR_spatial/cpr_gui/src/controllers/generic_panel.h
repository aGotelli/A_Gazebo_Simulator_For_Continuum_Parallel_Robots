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




#ifndef GENERIC_PANEL_H
#define GENERIC_PANEL_H

#include <QWidget>

#include <QtWidgets/QGroupBox>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QApplication>

#ifdef ROS
#include "src/controllers/generic_controller.h"
#else
#include "generic_controller.h"
#endif

#include <math.h>



class GenericPanel : public QWidget
{
   Q_OBJECT
public:
    explicit GenericPanel(QWidget *parent_widget=nullptr) : QWidget(parent_widget){}
    GenericPanel(QWidget *parent_widget,
                 const std::string &panel_title,
                 const QRect &panel_dimensions,
                 const std::string &main_windows_object_name="MainWindow");
    virtual ~GenericPanel(){}


    virtual void Update(){}


public slots:
    inline virtual void SetDisabled() { panel_group_box->setDisabled(true); }

    inline virtual void SetEnabled() { panel_group_box->setEnabled(true); }

    inline virtual void ResetPanel() { SetDisabled(); }

protected:

    QGroupBox *panel_group_box { nullptr };
    QGridLayout *panel_layout { nullptr };

    bool has_changed { false };

    void ErrorLogThisPanel(const std::string &error_message);

};



#endif // GENERIC_PANEL_H
