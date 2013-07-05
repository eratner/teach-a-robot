#include "pr2_simple_simulator/pr2_simple_simulator.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simple_sim");

  pr2_simple_simulator::PR2SimpleSimulator simple_sim;

  simple_sim.run();

  return 0;
}
