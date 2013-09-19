#include <dviz_local/demonstration_visualizer_client.h>

#include <QApplication>

int main(int argc, char **argv)
{
  QApplication a(argc, argv);

  demonstration_visualizer::DemonstrationVisualizerClient *dviz_client = 
    new demonstration_visualizer::DemonstrationVisualizerClient(argc, argv);
  dviz_client->show();
  
  a.exec();

  delete dviz_client;

  return 0;
}
