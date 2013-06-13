#include "pr2_motion_recorder.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pr2_motion_rec");

  PR2MotionRecorder recorder;

  recorder.run();

  return 0;
}
