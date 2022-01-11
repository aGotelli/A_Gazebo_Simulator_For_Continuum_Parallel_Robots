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




#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>
#include <QtWidgets/QFileDialog>
#include <QtWidgets/QMessageBox>

#include <thread>
#include <memory>
#include <iostream>
#include <yaml-cpp/yaml.h>


#include "menubar.h"

#ifdef ROS
#include <ros/ros.h>
#include <ros/package.h>

#include <std_srvs/Empty.h>

#include "src/distal_plate_pose_panel/distal_plate_pose_panel.h"
#include "src/distal_plate_wrench_panel/distal_plate_wrench_panel.h"
#include "src/joints_control_panel/joints_control_panel.h"
#include "src/solver_options_panel/solver_options_panel.h"

#include "src/parameters_vector_manager/parameters_vector_loader.h"
#include "src/parameters_vector_manager/parameters_vector_saving.h"

#include "src/dof_selection_window/dof_selection_window.h"

#include "cpr_messages/DGMParams.h"
#include "cpr_messages/IGMParams.h"
#include "cpr_messages/GuessVectorHandling.h"
#else


#include "solver_options_panel.h"
#include "distal_plate_pose_panel.h"
#include "distal_plate_wrench_panel.h"
#include "joints_control_panel.h"

#include "dof_selection_window.h"
#include "parameters_vector_loader.h"
#include "parameters_vector_saving.h"
#endif

using namespace std;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
#ifdef ROS
    explicit MainWindow(ros::NodeHandle &nh_);
#else
    MainWindow(const QApplication &app);
#endif


    virtual void closeEvent(QCloseEvent *close_event)override;

private slots:
    void RemoveRobot();

    void LoadRobot();

    void SaveParameters();

    void LoadParameters();

private:
    QWidget *centralWidget { nullptr };


    unsigned int number_of_limbs { 0 };

    MenuBar *menu_bar;

    SolverOptionsPanel *solver_options_panel { nullptr };

    JointsControlPanel *joints_control_panel { nullptr };

    DistalPlatePosePanel *distal_plate_pose_panel { nullptr };

    DistalPlateWrenchPanel *distal_plate_wrench_panel { nullptr };


    DofSelectionWindow *dof_selection_window { nullptr };


    QString robot_file_name;
    ParametersVectorLoader *parameters_vector_loader;
    ParametersVectorSaving *parameters_vector_saving;

    void SpawnRobot(const YAML::Node &node);

    void ResolveForUnderActuatedRobot(const YAML::Node &node);


    bool igm_problem { false };
    bool dgm_problem { false };

    bool robot_is_loaded { false };

    bool finished { false };

#ifdef ROS
    ros::NodeHandle nh;


    ros::Publisher igm_parameters_streamer;
    cpr_messages::IGMParams igm_parameters;

    ros::Publisher dgm_parameters_streamer;
    cpr_messages::DGMParams dgm_parameters;


    std::thread loop;

    void Update();
#endif
};

#endif // MAIN_WINDOW_H
