/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef DEMONSTRATION_VISUALIZER_CORE_H
#define DEMONSTRATION_VISUALIZER_CORE_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <interactive_markers/interactive_marker_server.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <interactive_markers/tools.h>

#include <dviz_core/Goal.h>
#include <dviz_core/Task.h>
#include <dviz_core/Command.h>
#include <dviz_core/visualization_helpers.h>
#include <dviz_core/demonstration_scene_manager.h>
#include <dviz_core/pr2_simulator.h>
#include <dviz_core/object_manager.h>
#include <cmath>

#include <QThread>
#include <QEvent>

namespace demonstration_visualizer {

/**
 * @brief This runs all the ROS functionality (i.e. publishing to topics, subscribing
 *        to services, etc.) on a separate thread from the dviz Qt component.
 *        In this way, the ROS loop and Qt event loop are separated.
 */
class DemonstrationVisualizerCore : public QThread
{
Q_OBJECT
public:
  DemonstrationVisualizerCore(int argc, char **argv, bool threaded = true);

  virtual ~DemonstrationVisualizerCore();

  bool init(int argc, char **argv);

  std::string getWorldFrame() const;

  bool processCommand(dviz_core::Command::Request &, dviz_core::Command::Response &);

  /**
   * @see PR2Simulator::pause().
   */
  void pauseSimulator();

  /**
   * @see PR2Simulator::pauseLater().
   */
  void pauseSimulatorLater();
  
  /**
   * @see PR2Simulator::play().
   */
  void playSimulator();

  void run();

  void updateGoals();

  void setRobotSpeed(double, double);

  void resetRobot();

  DemonstrationSceneManager *getSceneManager();

  MotionRecorder *getMotionRecorder();

  interactive_markers::InteractiveMarkerServer *getInteractiveMarkerServer();
  
  void processKeyEvent(int key, int type);
  
  void prepGripperForGoal(int goal_number);

  void setGripperOrientationControl(bool enabled);

  void setIgnoreCollisions(bool ignore);

  /**
   * Note that all poses are given in the world frame, if not 
   * otherwise specified.
   */
  geometry_msgs::Pose getBasePose();

  geometry_msgs::Pose getEndEffectorPose();

  geometry_msgs::Pose getEndEffectorPoseInBase();

  geometry_msgs::Pose getEndEffectorMarkerPose();

  void setBaseCommand(const geometry_msgs::Pose &);

  bool setEndEffectorGoalPose(const geometry_msgs::Pose &);

  void setBaseVelocity(const geometry_msgs::Twist &);

  void setJointStates(const sensor_msgs::JointState &);

  void showBasePath(const std::string &filename = "");

  void showInteractiveGripper(int goal_number);

  void graspMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void disableRobotMarkerControl();

  void enableRobotMarkerControl();

Q_SIGNALS:
  void rosShutdown();
  void goalComplete(int);
  void updateCamera(const geometry_msgs::Pose &, const geometry_msgs::Pose &);

private:
  DemonstrationSceneManager *demonstration_scene_manager_;
  ObjectManager* object_manager_;
  MotionRecorder *recorder_;
  PViz *pviz_;
  PR2Simulator *simulator_;

  ros::Publisher marker_pub_;

  std::string world_frame_;

  ros::Publisher end_effector_vel_cmd_pub_;
  ros::Publisher base_vel_cmd_pub_;
  ros::Publisher task_pub_;

  ros::ServiceServer command_service_;

  interactive_markers::InteractiveMarkerServer *int_marker_server_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_VISUALIZER_CORE_H