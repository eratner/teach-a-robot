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
#include <pr2_simple_simulator/KeyEvent.h>
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

namespace demonstration_visualizer {

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

  bool pauseSimulator(std_srvs::Empty &srv);
  
  bool playSimulator(std_srvs::Empty &srv);

  void run();

  void setRobotSpeed(double, double);

  void resetRobot();

  DemonstrationSceneManager *getSceneManager();
  
  void updateEndEffectorPose(const geometry_msgs::PoseStamped &pose);

  void processKeyEvent(int key, int type);

  void showBasePath(const std::string &filename = "");

  void updateBasePose(const geometry_msgs::PoseStamped &);
  
  geometry_msgs::Pose getBasePose() const;

  void sendBaseCommand(const geometry_msgs::Pose &);

  void sendBaseVelocityCommand(const geometry_msgs::Twist &);

  void updateEndEffectorMarkerPose(const geometry_msgs::Pose &);

Q_SIGNALS:
  void rosShutdown();
  void goalComplete(int);
  void updateCamera(const geometry_msgs::Pose &, const geometry_msgs::Pose &);

private:
  DemonstrationSceneManager *demonstration_scene_manager_;

  geometry_msgs::Pose end_effector_pose_;
  geometry_msgs::Pose base_pose_;
  geometry_msgs::Pose end_effector_marker_pose_;

  std::string global_frame_;

  ros::ServiceClient begin_recording_client_;
  ros::ServiceClient end_recording_client_;
  ros::ServiceClient begin_replay_client_;
  ros::ServiceClient end_replay_client_;
  ros::ServiceClient pause_simulator_client_;
  ros::ServiceClient play_simulator_client_;

  ros::ServiceClient reset_robot_client_;
  ros::ServiceClient set_robot_speed_client_;

  ros::ServiceClient key_event_client_;

  ros::ServiceClient show_base_path_client_;

  ros::ServiceClient set_base_command_client_;

  ros::Publisher end_effector_vel_cmd_pub_;
  ros::Publisher end_effector_marker_vel_pub_;
  ros::Publisher base_vel_cmd_pub_;

  ros::Subscriber end_effector_pose_sub_;
  ros::Subscriber base_pose_sub_;
  ros::Subscriber end_effector_marker_pose_sub_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_VISUALIZER_NODE_H
