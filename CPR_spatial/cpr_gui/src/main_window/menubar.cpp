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


