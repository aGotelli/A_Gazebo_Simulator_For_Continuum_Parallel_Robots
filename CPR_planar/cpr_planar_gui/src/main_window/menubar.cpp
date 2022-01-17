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



#include "menubar.h"

MenuBar::MenuBar(QMainWindow *parent_window)
    : QWidget(parent_window->centralWidget())
{
    menuBar = new QMenuBar(parent_window->centralWidget());
    menuBar->setObjectName(QString::fromUtf8("menuBar"));
    menuBar->setGeometry(QRect(0, 0, 635, 25));

    menuFile = new QMenu(menuBar);
    menuFile->setObjectName(QString::fromUtf8("menuFile"));
    menuFile->setTitle(QApplication::translate("MainWindow", "File", nullptr));
    menuBar->addAction(menuFile->menuAction());

    menuRobot = new QMenu(menuBar);
    menuRobot->setObjectName(QString::fromUtf8("menuRobot"));
    menuRobot->setTitle(QApplication::translate("MainWindow", "Robot", nullptr));
    menuBar->addAction(menuRobot->menuAction());

    parent_window->setMenuBar(menuBar);


    actionLoad = new QAction(this);
    actionLoad->setObjectName(QString::fromUtf8("actionLoad"));
    actionLoad->setText(QApplication::translate("MainWindow", "Load", nullptr));
    menuRobot->addAction(actionLoad);

    actionDelete = new QAction(this);
    actionDelete->setObjectName(QString::fromUtf8("actionDelete"));
    actionDelete->setText(QApplication::translate("MainWindow", "Delete", nullptr));
    menuRobot->addAction(actionDelete);

    QObject::connect(actionLoad, &QAction::triggered, this, &MenuBar::LoadRobot);

    QObject::connect(actionDelete, &QAction::triggered, this, &MenuBar::DeleteRobot);



    action_save_parameters = new QAction(this);
    action_save_parameters->setObjectName(QString::fromUtf8("action_save_parameters"));
    action_save_parameters->setText(QApplication::translate("MainWindow", "Save Parameters", nullptr));
    menuFile->addAction(action_save_parameters);

    action_load_parameters = new QAction(this);
    action_load_parameters->setObjectName(QString::fromUtf8("action_load_parameters"));
    action_load_parameters->setText(QApplication::translate("MainWindow", "Load Parameters", nullptr));
    menuFile->addAction(action_load_parameters);

    QObject::connect(action_save_parameters, &QAction::triggered, this, &MenuBar::SaveParameters);

    QObject::connect(action_load_parameters, &QAction::triggered, this, &MenuBar::LoadParameters);

}


