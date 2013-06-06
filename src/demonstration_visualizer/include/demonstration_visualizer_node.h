#ifndef DEMONSTRATION_VISUALIZER_NODE_H
#define DEMONSTRATION_VISUALIZER_NODE_H

#include <QThread>
#include <ros/ros.h>

#include <pr2_motion_recorder/FilePath.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>

#include <string>

class DemonstrationVisualizerNode : public QThread
{
Q_OBJECT
public:
  DemonstrationVisualizerNode(int argc, char **argv);

  virtual ~DemonstrationVisualizerNode();

  bool init(int argc, char **argv);

  bool beginRecording(pr2_motion_recorder::FilePath &srv);

  bool endRecording(std_srvs::Empty &srv);

  bool beginReplay(pr2_motion_recorder::FilePath &srv);

  bool endReplay(std_srvs::Empty &srv);

  void publishVisualizationMarker(const visualization_msgs::Marker &msg,
				  bool interactive_marker = false);

  void run();

  void processInteractiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

Q_SIGNALS:
  void rosShutdown();

private:
  ros::ServiceClient begin_recording_client_;
  ros::ServiceClient end_recording_client_;
  ros::ServiceClient begin_replay_client_;
  ros::ServiceClient end_replay_client_;

  ros::Publisher marker_pub_;

  interactive_markers::InteractiveMarkerServer *interactive_marker_server_;

};

#endif // DEMONSTRATION_VISUALIZER_NODE_H
