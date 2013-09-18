#include <dviz_core/demonstration_visualizer_core.h>

int main(int argc, char **argv)
{
  demonstration_visualizer::DemonstrationVisualizerCore dviz_core(argc, argv, false);

  dviz_core.run();

  return 0;
}
