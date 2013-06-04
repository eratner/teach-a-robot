#include "pr2_motion_recorder.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_recorder");

  PR2MotionRecorder recorder;

  recorder.run();

  return 0;
}
