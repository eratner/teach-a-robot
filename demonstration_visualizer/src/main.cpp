#include "demonstration_visualizer/demonstration_visualizer.h"

#include <QApplication>

int main(int argc, char **argv)
{
  QApplication a(argc, argv);

  demonstration_visualizer::DemonstrationVisualizer *viz = 
    new demonstration_visualizer::DemonstrationVisualizer(argc, argv);
  viz->show();
  
  a.exec();

  delete viz;

  return 0;
}
