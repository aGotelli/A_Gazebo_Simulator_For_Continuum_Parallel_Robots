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




#include "solver_options_panel.h"

SolverOptionsPanel::SolverOptionsPanel(QWidget *parent_widget,
                                       const std::string &panel_title,
                                       const QRect &panel_dimensions,
                                       const std::string &main_windows_object_name)
                : GenericPanel(parent_widget,
                               panel_title,
                               panel_dimensions,
                               main_windows_object_name)
{
    buttons_grid_layout = new QGridLayout();
    buttons_grid_layout->setSpacing(6);
    buttons_grid_layout->setObjectName(QString::fromUtf8("solver_buttons_gridLayout"));

    igm_radioButton = new QRadioButton(panel_group_box);
    igm_radioButton->setText(QApplication::translate("MainWindow", "IGM", nullptr));
    igm_radioButton->setObjectName(QString::fromUtf8("igm_radioButton"));
    buttons_grid_layout->addWidget(igm_radioButton, 0, 0, 1, 1);

    dgm_radioButton = new QRadioButton(panel_group_box);
    dgm_radioButton->setObjectName(QString::fromUtf8("dgm_radioButton"));
    dgm_radioButton->setText(QApplication::translate("MainWindow", "DGM", nullptr));
    buttons_grid_layout->addWidget(dgm_radioButton, 1, 0, 1, 1);

    QObject::connect(igm_radioButton, &QRadioButton::clicked, this, &SolverOptionsPanel::IGMCalled);
    QObject::connect(dgm_radioButton, &QRadioButton::clicked, this, &SolverOptionsPanel::DGMCalled);

    panel_layout->addLayout(buttons_grid_layout, 0, 0);

    SetDisabled();
}





void SolverOptionsPanel::ResetPanel()
{
    UnCheckRadioButton(igm_radioButton);

    UnCheckRadioButton(dgm_radioButton);
}


void SolverOptionsPanel::UnCheckRadioButton(QRadioButton *radioButton)
{
    radioButton->setAutoExclusive(false);
    radioButton->setChecked(false);
    radioButton->setAutoExclusive(true);
}
