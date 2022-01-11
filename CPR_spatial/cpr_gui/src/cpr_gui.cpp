#include <ros/ros.h>
#include "main_window/main_window.h"

#include <QApplication>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpr_gui");
  ros::NodeHandle nh;

  QApplication app(argc, argv);

  MainWindow mw(nh);
  mw.show();

  app.exec();



  ROS_INFO("The GUI has finished");
}
