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




#ifndef SOLVER_OPTIONS_PANEL_H
#define SOLVER_OPTIONS_PANEL_H



#ifdef ROS
#include "src/controllers/generic_panel.h"
#else
#include "generic_panel.h"
#endif

class SolverOptionsPanel : public GenericPanel
{
    Q_OBJECT
public:
    explicit SolverOptionsPanel(QWidget *parent_widget=nullptr) : GenericPanel(parent_widget){}
    SolverOptionsPanel(QWidget *parent_widget,
                       const std::string &panel_title,
                       const QRect &panel_dimensions,
                       const std::string &main_windows_object_name="MainWindow");

    virtual ~SolverOptionsPanel()override{}
signals:
    void IGMCalled();

    void DGMCalled();

public slots:

    virtual void ResetPanel()override;

private:
    QGridLayout *buttons_grid_layout { nullptr };
    QRadioButton *igm_radioButton { nullptr };
    QRadioButton *dgm_radioButton { nullptr };

    void UnCheckRadioButton(QRadioButton *radioButton);
};

#endif // SOLVER_OPTIONS_PANEL_H
