#ifndef DEMONSTRATION_VISUALIZER_USER_H
#define DEMONSTRATION_VISUALIZER_USER_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include <dviz_core/pr2_simulator.h>
#include <dviz_core/object_manager.h>
#include <dviz_core/motion_recorder.h>
#include <dviz_core/common.h>
#include <dviz_core/Command.h>

namespace demonstration_visualizer
{

class DemonstrationVisualizerUser
{
public:
  DemonstrationVisualizerUser(int argc, char **argv, int id);

  ~DemonstrationVisualizerUser();

  /**
   * @brief The main run loop.
   */
  void run();

  /**
   * @brief The service callback for processing Command messages.
   *
   * @return true on successful execution of the command, false otherwise.
   */
  bool processCommand(dviz_core::Command::Request &, dviz_core::Command::Response &);

private:
  /**
   * @brief Initializes ROS and names the DVizUser node with its
   *        unique identifier.
   *
   * @return true on success; false on failure.
   */
  bool init(int argc, char **argv);

  int id_;
  bool ok_;
  ros::ServiceServer command_service_;

  interactive_markers::InteractiveMarkerServer *int_marker_server_;
  PViz *pviz_;
  ObjectManager *object_manager_;
  MotionRecorder *recorder_;
  PR2Simulator *simulator_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_VISUALIZER_USER_H
