#include <QApplication>
#include <ros/ros.h>
#include "demonstration_visualizer.h"

int main(int argc, char **argv)
{
  QApplication a(argc, argv);

  DemonstrationVisualizer *viz = new DemonstrationVisualizer(argc, argv);
  viz->show();

  a.exec();

  delete viz;

  return 0;
}
