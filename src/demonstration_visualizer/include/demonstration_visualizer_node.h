#ifndef DEMONSTRATION_VISUALIZER_NODE_H
#define DEMONSTRATION_VISUALIZER_NODE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pr2_motion_recorder/FilePath.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "visualization_helpers.h"
#include "base_movement_controller.h"
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

  void updateLatestBasePose(const geometry_msgs::PoseWithCovarianceStamped &);

  void processBaseMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void updateBaseMarker();

  void moveBaseToGoal();

Q_SIGNALS:
  void rosShutdown();
  void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

private:
  std::string global_frame_;

  ros::ServiceClient begin_recording_client_;
  ros::ServiceClient end_recording_client_;
  ros::ServiceClient begin_replay_client_;
  ros::ServiceClient end_replay_client_;

  ros::Publisher marker_pub_;

  interactive_markers::InteractiveMarkerServer *interactive_marker_server_;
  interactive_markers::InteractiveMarkerServer *base_marker_server_;
  ros::Subscriber base_pose_sub_;
  geometry_msgs::PoseStamped latest_base_pose_;
  geometry_msgs::PoseStamped base_goal_pose_;

  ros::Publisher base_cmd_vel_pub_;
  bool base_rotation_done_;
  bool base_translation_done_;

  BaseMovementController base_movement_controller_;

};

#endif // DEMONSTRATION_VISUALIZER_NODE_H
