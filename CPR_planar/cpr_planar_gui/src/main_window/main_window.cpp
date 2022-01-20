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
#include "src/main_window/main_window.h"
#else
#include "main_window.h"
#endif


#ifdef ROS
MainWindow::MainWindow(ros::NodeHandle &nh_) :
    QMainWindow(nullptr),
    nh(nh_)
#else
MainWindow::MainWindow(const QApplication &app) :
    QMainWindow(nullptr)
#endif
{
    if (this->objectName().isEmpty())
        this->setObjectName(QString::fromUtf8("MainWindow"));
    this->resize(700, 500);
    this->setWindowTitle(QApplication::translate("MainWindow", "CRP Simulator", nullptr));

    QString path_to_icon;
#ifdef ROS
    path_to_icon = QString::fromStdString( ros::package::getPath("cpr_planar_gui") );
    path_to_icon.append( QString("/src/main_window/icon.png") );
#else
    path_to_icon = app.applicationDirPath();
    //  This will be the build directory, we will go back to the src
    path_to_icon.truncate( path_to_icon.lastIndexOf(QChar('/')) );  //  mainwindow
    path_to_icon.truncate( path_to_icon.lastIndexOf(QChar('/')) );  //  src
    path_to_icon.truncate( path_to_icon.lastIndexOf(QChar('/')) );  //  robot_gui

    //  Add path to the icon
    path_to_icon.append( QString("/src/main_window/icon.png") );
#endif
    this->setWindowIcon( QIcon(path_to_icon) );

    centralWidget = new QWidget(this);
    centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
    this->setCentralWidget(centralWidget);

    menu_bar = new MenuBar(this);
    QObject::connect(menu_bar, &MenuBar::LoadRobot, this, &MainWindow::LoadRobot);
    QObject::connect(menu_bar, &MenuBar::DeleteRobot, this, &MainWindow::RemoveRobot);
    QObject::connect(menu_bar, &MenuBar::SaveParameters, this, &MainWindow::SaveParameters);
    QObject::connect(menu_bar, &MenuBar::LoadParameters, this, &MainWindow::LoadParameters);

    solver_options_panel = new SolverOptionsPanel(centralWidget,
                                                  "Solver Options",
                                                  QRect(30, 30, 100, 100));
    QObject::connect(solver_options_panel, &SolverOptionsPanel::DGMCalled, [this](){
      this->dgm_problem = true;
      this->igm_problem = false;

      this->distal_plate_pose_panel->SetDisabled();
      this->joints_control_panel->SetEnabled();
    });

    QObject::connect(solver_options_panel, &SolverOptionsPanel::IGMCalled, [this](){
      this->igm_problem = true;
      this->dgm_problem = false;

      this->distal_plate_pose_panel->SetEnabled();
      this->joints_control_panel->SetDisabled();
    });

#ifdef ROS
    joints_control_panel = new JointsControlPanel(centralWidget,
                                                  "Joints Control",
                                                  QRect(10, 220, 350, 190),
                                                  nh);
#else
    joints_control_panel = new JointsControlPanel(centralWidget,
                                                  "Joints Control",
                                                  QRect(10, 220, 350, 190));
#endif

#ifdef ROS
    distal_plate_pose_panel = new DistalPlatePosePanel(centralWidget,
                                                       "Distal Plate Pose",
                                                       QRect(150, 30, 500, 180),
                                                       nh);
#else
    distal_plate_pose_panel = new DistalPlatePosePanel(centralWidget,
                                                       "Distal Plate Pose",
                                                       QRect(150, 30, 500, 180));
#endif

    distal_plate_wrench_panel = new DistalPlateWrenchPanel(centralWidget,
                                                           "Distal Plate Wrench",
                                                           QRect(380, 220, 270, 190));


    QMetaObject::connectSlotsByName(this);











#ifdef ROS
    loop = std::thread([&](){Update();});
#endif

}


void MainWindow::closeEvent (QCloseEvent *close_event)
{
    if(finished) {
      close_event->accept();
      return;
    }

    if(!robot_is_loaded) {
      close_event->accept();
      return;
    }

    QMessageBox::StandardButton resBtn = QMessageBox::question( this, "Closing Robot GUI",
                                                                tr("Do you want to save the last parameters vector?\n"),
                                                                QMessageBox::Cancel | QMessageBox::No | QMessageBox::Yes,
                                                                QMessageBox::No);
    if(resBtn == QMessageBox::No)
      close_event->accept();

    if(resBtn == QMessageBox::Cancel)
      close_event->ignore();

    if(resBtn == QMessageBox::Yes){
      SaveParameters();
      close_event->ignore();
      QObject::connect(parameters_vector_saving, &ParametersVectorSaving::Saved, [this](){
        this->finished = true;

        this->close();
      });
    }

}


void MainWindow::RemoveRobot()
{
#ifdef ROS
    ros::ServiceClient pause_gazebo_client = nh.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty::Request preq;
    std_srvs::Empty::Response pres;
    pause_gazebo_client.call(preq, pres);



    ros::ServiceClient remove_robot_client = nh.serviceClient<std_srvs::Empty>("/remove_robot");

    std_srvs::EmptyRequest req;
    std_srvs::EmptyResponse res;
    remove_robot_client.call(req, res);
#endif

    solver_options_panel->ResetPanel();
    solver_options_panel->SetDisabled();

    distal_plate_pose_panel->SetDisabled();

    distal_plate_wrench_panel->SetDisabled();

    joints_control_panel->ResetPanel();
    joints_control_panel->SetDisabled();

    robot_is_loaded = false;

}


void MainWindow::LoadRobot()
{
    QString path;
#ifdef ROS
    path = QString::fromStdString( ros::package::getPath("cpr_planar_physics") );
    path.append( QString("/launch") );
#else
    path = QString("/home/andrea/QtCreator/latest/catkin/src/A-Gazebo-Simulator-for-Continuum-Parallel-Robots/cpr_planar_physics/launch");
#endif

    robot_file_name = QFileDialog::getOpenFileName(this,
                                                   tr("Open A Robot Configuration File"),
                                                   path,
                                                   tr("YAML File (*.yaml)"));


    if (robot_file_name.isEmpty())
        return;

    QFile robot_configurations_file(robot_file_name);

    if (!robot_configurations_file.open(QIODevice::ReadOnly)) {
        QMessageBox::information(this, tr("Unable to open file"),
            robot_configurations_file.errorString());
        return;
    }

    YAML::Node node = YAML::LoadFile( robot_file_name.toStdString() );

    //  Read the number of limbs from the .yaml file
    number_of_limbs = node["number_of_limbs"].as<unsigned int>();

    if(number_of_limbs<3)
      ResolveForUnderActuatedRobot(node);
    else SpawnRobot(node);



}



void MainWindow::SpawnRobot(const YAML::Node &node)
{

  YAML::Node launching = node["launching"];

  const std::string package_name = launching["package_name"].as<std::string>();

  std::vector<std::string> launch_files(launching["launch_files"].as<std::vector<std::string>>());
#ifdef ROS
  for(auto launch_file : launch_files) {
    if(launch_file.find("launch") == std::string::npos)
      launch_file.append(".launch");

    const std::string command = "roslaunch " + package_name + " " + launch_file;


    system(command.data());

  }
#endif
  joints_control_panel->CreateJointWidgets(number_of_limbs);

  robot_is_loaded = true;

  joints_control_panel->SetEnabled();

  solver_options_panel->SetEnabled();

  distal_plate_pose_panel->SetEnabled();

  distal_plate_wrench_panel->SetEnabled();

  //  Unpause Gazebo directly after all the spawns
#ifdef ROS
  std_srvs::EmptyRequest req;
  std_srvs::EmptyResponse res;
  ros::ServiceClient unpause_gazebo_client = nh.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
  unpause_gazebo_client.call(req, res);
#endif
}

void MainWindow::ResolveForUnderActuatedRobot(const YAML::Node &node)
{
    dof_selection_window = new DofSelectionWindow(3-number_of_limbs);

    QObject::connect(dof_selection_window,
                   &DofSelectionWindow::DistalPlateDOFsDecided,
                   [this, node](const std::vector<bool> &underactuation_vector){

        this->distal_plate_pose_panel->DisableOnlyOverconstrains(underactuation_vector);

#ifdef ROS
        nh.setParam("/underactuation_vector", underactuation_vector);
#endif

        this->distal_plate_pose_panel->SetEnabled();
        this->distal_plate_wrench_panel->SetEnabled();
        this->joints_control_panel->SetEnabled();
        this->solver_options_panel->SetEnabled();

        this->SpawnRobot(node);
    });



  dof_selection_window->show();

}

void MainWindow::SaveParameters()
{
  if(!robot_is_loaded)
    return;

  std::vector<float> parameters;

#ifdef ROS
  ros::ServiceClient robot_parameters_client = nh.serviceClient<cpr_planar_messages::GuessVectorHandling>("/parameters_vector_services");

  cpr_planar_messages::GuessVectorHandlingRequest req;
  cpr_planar_messages::GuessVectorHandlingRequest res;

  if(!robot_parameters_client.call(req, res))
   cout << "Not possible to get parameters from the robot" << endl;

  parameters = res.given_guess;
#endif

  parameters_vector_saving = new ParametersVectorSaving(robot_file_name, parameters);
  parameters_vector_saving->show();
}



void MainWindow::LoadParameters()
{
  if(!robot_is_loaded)
    return;

  parameters_vector_loader = new ParametersVectorLoader(robot_file_name);
  parameters_vector_loader->show();

  QObject::connect(parameters_vector_loader,
                   &ParametersVectorLoader::ParametersVectorChosen,
                   [this](const std::vector<QVariant> parameters_vector){
#ifdef ROS
    ros::ServiceClient robot_parameters_client = this->nh.serviceClient<cpr_planar_messages::GuessVectorHandling>("/parameters_vector_services");

    cpr_planar_messages::GuessVectorHandlingRequest req;
    cpr_planar_messages::GuessVectorHandlingRequest res;

    req.given_guess.resize(parameters_vector.size());

    for(unsigned int i=0; i<parameters_vector.size(); i++)
      req.given_guess[i] = parameters_vector[i].toFloat();

    robot_parameters_client.call(req, res);
    distal_plate_pose_panel->Update();


#else
    cout << "Selected parameters vector : " << endl;
    for(const auto& parameter : parameters_vector)
        cout << parameter.toFloat() << ", ";
    cout << endl << endl;
#endif
  });
}






#ifdef ROS
void MainWindow::Update()
{
  igm_parameters_streamer = nh.advertise<cpr_planar_messages::IGMParameters>("/igm_parameters", 1);
  dgm_parameters_streamer = nh.advertise<cpr_planar_messages::DGMParameters>("/dgm_parameters", 1);

  ros::Rate rate(20);
  while(ros::ok()){
    ros::spinOnce();

    if(!robot_is_loaded) {
      rate.sleep();
      continue;
    }


    if(dgm_problem){

      if(joints_control_panel->GetJointValuesIfChanged(dgm_parameters.joints_values))
        dgm_parameters_streamer.publish(dgm_parameters);

      if(distal_plate_wrench_panel->GetWrenchIfChanged(dgm_parameters.wrench))
        dgm_parameters_streamer.publish(dgm_parameters);

    } else joints_control_panel->Update();


    if(igm_problem){

      if(distal_plate_pose_panel->GetPoseIfChanged(igm_parameters.pose))
        igm_parameters_streamer.publish(igm_parameters);

      if(distal_plate_wrench_panel->GetWrenchIfChanged(igm_parameters.wrench))
        igm_parameters_streamer.publish(igm_parameters);

      //cout << "Distal Plate Position : " << endl << igm_parameters.distal_plate_pose.position.x << endl << igm_parameters.distal_plate_pose.position.y << endl << igm_parameters.distal_plate_pose.position.z << endl << endl << endl;

    } else distal_plate_pose_panel->Update();



    rate.sleep();
  }
}
#endif
