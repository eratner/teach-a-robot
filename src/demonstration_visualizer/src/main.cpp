#include <QApplication>
#include <ros/ros.h>
#include "demonstration_visualizer.h"

int main(int argc, char **argv)
{
  if(!ros::isInitialized())
  {
    ros::init(argc, argv, "demovis");
  }

  QApplication a(argc, argv);

  DemonstrationVisualizer *viz = new DemonstrationVisualizer();
  viz->show();

  a.exec();

  delete viz;
  viz = 0;

  return 0;
}
