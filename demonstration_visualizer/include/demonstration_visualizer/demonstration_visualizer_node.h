#ifndef DEMONSTRATION_VISUALIZER_NODE_H
#define DEMONSTRATION_VISUALIZER_NODE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pr2_motion_recorder/FilePath.h>
#include <pr2_simple_simulator/SetSpeed.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/tools.h>

#include "demonstration_visualizer/visualization_helpers.h"
#include "demonstration_visualizer/demonstration_scene_manager.h"
#include <cmath>

#include <QThread>

class DemonstrationVisualizerNode : public QThread
{
Q_OBJECT
public:
  DemonstrationVisualizerNode(int argc, char **argv);

  virtual ~DemonstrationVisualizerNode();

  bool init(int argc, char **argv);

  std::string getGlobalFrame() const;

  bool beginRecording(pr2_motion_recorder::FilePath &srv);

  bool endRecording(std_srvs::Empty &srv);

  bool beginReplay(pr2_motion_recorder::FilePath &srv);

  bool endReplay(std_srvs::Empty &srv);

  void publishVisualizationMarker(const visualization_msgs::Marker &msg,
				  bool attach_interactive_marker = false);

  bool removeInteractiveMarker(const std::string &name);

  void clearInteractiveMarkers();

  void run();

  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void setRobotSpeed(double, double);

  void resetRobot();

  void updateTaskGoals();

  void processGoalFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  DemonstrationSceneManager *getSceneManager();

  void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

Q_SIGNALS:
  void rosShutdown();

private:
  DemonstrationSceneManager *demonstration_scene_manager_;
  bool edit_goals_mode_;

  std::string global_frame_;

  ros::ServiceClient begin_recording_client_;
  ros::ServiceClient end_recording_client_;
  ros::ServiceClient begin_replay_client_;
  ros::ServiceClient end_replay_client_;

  ros::ServiceClient reset_robot_client_;
  ros::ServiceClient set_robot_speed_client_;

  ros::Publisher marker_pub_;

  interactive_markers::InteractiveMarkerServer *interactive_marker_server_;

};

#endif // DEMONSTRATION_VISUALIZER_NODE_H
