#include <dviz_core/demonstration_visualizer.h>

int main(int argc, char **argv)
{
  demonstration_visualizer::DemonstrationVisualizerCore dviz(argc, argv, false);

  dviz.run();

  return 0;
}
