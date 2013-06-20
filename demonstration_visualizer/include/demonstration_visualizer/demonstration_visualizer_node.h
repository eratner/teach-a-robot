/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef DEMONSTRATION_VISUALIZER_NODE_H
#define DEMONSTRATION_VISUALIZER_NODE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <pr2_simple_simulator/FilePath.h>
#include <pr2_simple_simulator/SetSpeed.h>
#include <pr2_simple_simulator/SetPose.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/tools.h>

#include "demonstration_visualizer/visualization_helpers.h"
#include "demonstration_visualizer/demonstration_scene_manager.h"
#include <cmath>

#include <QThread>
#include <QEvent>

/**
 * @brief This runs all the ROS functionality (i.e. publishing to topics, subscribing
 *        to services, etc.) on a separate thread from the dviz Qt component.
 *        In this way, the ROS loop and Qt event loop are separated.
 */
class DemonstrationVisualizerNode : public QThread
{
Q_OBJECT
public:
  DemonstrationVisualizerNode(int argc, char **argv);

  virtual ~DemonstrationVisualizerNode();

  bool init(int argc, char **argv);

  std::string getGlobalFrame() const;

  bool beginRecording(pr2_simple_simulator::FilePath &srv);

  bool endRecording(std_srvs::Empty &srv);

  bool beginReplay(pr2_simple_simulator::FilePath &srv);

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

  void setEditGoalsMode(bool mode);
  
  void setCurrentGoal(int goal_number);

  void updateEndEffectorPose(const geometry_msgs::PoseStamped &pose);

  void processKeyEvent(int key, int type);

Q_SIGNALS:
  void rosShutdown();
  void goalComplete(int);
  void focusCameraTo(float, float, float);

private:
  DemonstrationSceneManager *demonstration_scene_manager_;
  bool edit_goals_mode_;
  int current_goal_;
  geometry_msgs::Pose end_effector_pose_;

  std::string global_frame_;

  ros::ServiceClient begin_recording_client_;
  ros::ServiceClient end_recording_client_;
  ros::ServiceClient begin_replay_client_;
  ros::ServiceClient end_replay_client_;

  ros::ServiceClient reset_robot_client_;
  ros::ServiceClient set_robot_speed_client_;

  ros::Publisher marker_pub_;
  ros::Publisher end_effector_vel_cmd_pub_;

  ros::Subscriber end_effector_pose_sub_;

  interactive_markers::InteractiveMarkerServer *interactive_marker_server_;

};

#endif // DEMONSTRATION_VISUALIZER_NODE_H
