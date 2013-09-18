#include <dviz_core/demonstration_visualizer_user.h>

int main(int argc, char **argv)
{
  int id = 0;
  if(argc >= 2)
  {
    id = atoi(argv[1]);
  }
  else
  {
    ROS_WARN("[DVizUser] No user ID provided, so using default (0).");
  }

  demonstration_visualizer::DemonstrationVisualizerUser dviz_user(argc, argv, id);

  dviz_user.run();

  return 0;
}
