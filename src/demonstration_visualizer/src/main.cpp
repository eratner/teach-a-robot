#include "demonstration_visualizer.h"

#include <QApplication>

int main(int argc, char **argv)
{
  QApplication a(argc, argv);

  DemonstrationVisualizer *viz = new DemonstrationVisualizer(argc, argv);
  viz->show();

  a.exec();

  delete viz;

  return 0;
}
