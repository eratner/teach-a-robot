/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef DEMONSTRATION_VISUALIZER_NODE_H
#define DEMONSTRATION_VISUALIZER_NODE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/tools.h>

#include "demonstration_visualizer/visualization_helpers.h"
#include "demonstration_visualizer/demonstration_scene_manager.h"
#include "demonstration_visualizer/pr2_simulator.h"
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

  std::string getWorldFrame() const;

  void pauseSimulator();
  
  void playSimulator();

  void run();

  void setRobotSpeed(double, double);

  void resetRobot();

  DemonstrationSceneManager *getSceneManager();

  MotionRecorder *getMotionRecorder();
  
  void processKeyEvent(int key, int type);
  
  /**
   * Note that all poses are given in the world frame.
   */
  geometry_msgs::Pose getBasePose();

  geometry_msgs::Pose getEndEffectorPose();

  geometry_msgs::Pose getEndEffectorMarkerPose();

  void sendBaseCommand(const geometry_msgs::Pose &);

  void sendBaseVelocityCommand(const geometry_msgs::Twist &);

  void setJointStates(const sensor_msgs::JointState &);

  void showBasePath(const std::string &filename = "");

  void showInteractiveGripper(const geometry_msgs::Pose &goal_pose, 
			      double distance = 0.25,
			      bool shadow = false);

Q_SIGNALS:
  void rosShutdown();
  void goalComplete(int);
  void updateCamera(const geometry_msgs::Pose &, const geometry_msgs::Pose &);

private:
  void runSimulator();

  DemonstrationSceneManager *demonstration_scene_manager_;
  MotionRecorder *recorder_;
  PR2Simulator *simulator_;

  ros::Publisher marker_pub_;

  std::string world_frame_;

  ros::Publisher end_effector_vel_cmd_pub_;
  ros::Publisher end_effector_marker_vel_pub_;
  ros::Publisher base_vel_cmd_pub_;

  interactive_markers::InteractiveMarkerServer *int_marker_server_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_VISUALIZER_NODE_H
