/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef PR2_SIMULATOR_H
#define PR2_SIMULATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <pviz/pviz.h>
#include <interactive_markers/interactive_marker_server.h>
#include <demonstration_visualizer/base_movement_controller.h>
#include <demonstration_visualizer/end_effector_controller.h>
#include <demonstration_visualizer/motion_recorder.h>
#include <demonstration_visualizer/object_manager.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>
#include <tf/transform_broadcaster.h>

#include <Qt>
#include <QEvent>

#include <vector>
#include <string>
#include <cmath>
#include <map>

namespace demonstration_visualizer {

class PR2Simulator
{
public:
  PR2Simulator(MotionRecorder *recorder, 
	       PViz *pviz, 
	       interactive_markers::InteractiveMarkerServer *int_marker_server,
	       ObjectManager* object_manager);

  ~PR2Simulator();

  void setFrameRate(double);

  double getFrameRate() const;

  void play();
  
  void pause();

  bool isPlaying() const;

  void run();
  
  /**
   * @brief Moves the robot (base and end-effectors) by first proposing new base and
   *        end-effector poses, checking these for validity (i.e. no collisions and 
   *        valid IK).
   */
  void moveRobot();

  /**
   * @brief Checks the validity of the robot's pose.
   */
  bool isValidRobotPose();

  /**
   * @brief Update the (linear/angular) velocity of the base.
   * @param[in] 
   */
  void updateVelocity(const geometry_msgs::Twist &);

  /**
   * @brief Visualizes the robot using PViz.
   */
  void visualizeRobot();

  /**
   * @brief Updates the pose of the end-effector in Cartesian space
   *        based on the current joint angles, using forward kinematics.
   */
  void updateEndEffectorPose();

  void setEndEffectorGoalPose(const geometry_msgs::Pose &);

  void updateEndEffectorVelocity(const geometry_msgs::Twist &);

  void baseMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void gripperMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void setSpeed(double, double);

  void resetRobot();

  void setRobotPose(const geometry_msgs::Pose &pose);

  void setJointStates(const sensor_msgs::JointState &joints);

  void setRobotBaseCommand(const geometry_msgs::Pose &command);

  /**
   * @brief Provides a way to notify the simulator of key events recieved
   *        elsewhere.
   */
  void processKeyEvent(int key, int type);

  void updateEndEffectorMarker();

  void updateEndEffectorMarkerVelocity(const geometry_msgs::Twist &);

  bool isBaseMoving() const;

  /**
   * @brief Checks to see if the given pose is a valid pose for the end effector
   *        using FK with the current joint angles.
   */
  bool isValidEndEffectorPose(const geometry_msgs::Pose &);

  bool validityCheck(const std::vector<double> &rangles, 
		     const std::vector<double> &langles, 
		     const BodyPose &bp);

  void attach(int id, KDL::Frame transform);

  void detach();

  void showEndEffectorWorkspaceArc();

  geometry_msgs::Pose getBasePose() const;

  geometry_msgs::Pose getEndEffectorPose();

  geometry_msgs::Pose getEndEffectorMarkerPose();

private:
  bool playing_;

  void updateTransforms();

  double frame_rate_;

  bool attached_object_;
  int attached_id_;
  KDL::Frame attached_transform_;

  ObjectManager* object_manager_;
  PViz *pviz_;
  interactive_markers::InteractiveMarkerServer *int_marker_server_;
  BaseMovementController base_movement_controller_;
  EndEffectorController end_effector_controller_;

  MotionRecorder *recorder_;

  ros::Subscriber vel_cmd_sub_;
  ros::Subscriber end_effector_vel_cmd_sub_;
  ros::Subscriber end_effector_marker_vel_sub_;
  
  ros::Publisher marker_pub_;

  geometry_msgs::Twist vel_cmd_;
  geometry_msgs::Twist key_vel_cmd_;
  geometry_msgs::Twist end_effector_vel_cmd_;
  geometry_msgs::Twist end_effector_marker_vel_;
  geometry_msgs::PoseStamped base_pose_;
  geometry_msgs::PoseStamped end_effector_pose_;
  geometry_msgs::PoseStamped end_effector_goal_pose_;
  sensor_msgs::JointState joint_states_;

  visualization_msgs::MarkerArray robot_markers_;

  geometry_msgs::PoseStamped goal_pose_;

  sbpl_arm_planner::KDLRobotModel kdl_robot_model_;
  KDL::Frame map_in_torso_lift_link_;
  std::map<std::string, int> joints_map_;

  bool is_moving_r_gripper_;

  tf::TransformBroadcaster tf_broadcaster_;

};

} // namespace demonstration_visualizer

#endif // PR2_SIMULATOR_H
