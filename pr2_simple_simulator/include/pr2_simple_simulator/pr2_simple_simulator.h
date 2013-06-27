/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef PR2_SIMPLE_SIMULATOR_H
#define PR2_SIMPLE_SIMULATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <pviz/pviz.h>
#include <interactive_markers/interactive_marker_server.h>
#include <pr2_simple_simulator/base_movement_controller.h>
#include <pr2_simple_simulator/end_effector_controller.h>
#include <pr2_simple_simulator/motion_recorder.h>
#include <pr2_simple_simulator/SetSpeed.h>
#include <pr2_simple_simulator/SetPose.h>
#include <pr2_simple_simulator/SetJoints.h>
#include <pr2_simple_simulator/FilePath.h>
#include <pr2_simple_simulator/KeyEvent.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>
#include <tf/transform_broadcaster.h>

#include <vector>
#include <string>
#include <cmath>
#include <map>

class PR2SimpleSimulator
{
public:
  PR2SimpleSimulator();

  ~PR2SimpleSimulator();

  void setFrameRate(double);

  double getFrameRate() const;

  void run();

  /**
   * @brief Update the (linear/angular) velocity of the base.
   * @param[in] 
   */
  void updateVelocity(const geometry_msgs::Twist &);

  /**
   * @brief Moves the base of the robot towards the current 
   *        (if any) goal pose.
   */
  void moveRobot();

  /**
   * @brief Moves the end-effectors according to the current
   *        end-effector velocity commands.
   */
  void moveEndEffectors();

  /**
   * @brief Visualizes the robot using PViz.
   */
  void visualizeRobot();

  /**
   * @brief Updates the pose of the end-effector in Cartesian space
   *        based on the current joint angles, using forward kinematics.
   */
  void updateEndEffectorPose();

  /**
   * @brief Finds and sets the appropriate positions of the
   *        (right) arm joints using IK so that the end-effector 
   *        is at the desired pose.
   * @param[in] the desired pose of the (right) end-effector.
   * @return true on success, otherwise false.
   */
  bool setEndEffectorPose(const geometry_msgs::Pose &);

  void setEndEffectorGoalPose(const geometry_msgs::Pose &);

  void updateEndEffectorVelocity(const geometry_msgs::Twist &);

  void baseMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void gripperMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  bool setSpeed(pr2_simple_simulator::SetSpeed::Request  &,
		   pr2_simple_simulator::SetSpeed::Response &);

  bool resetRobot(std_srvs::Empty::Request  &,
		  std_srvs::Empty::Response &);

  bool setRobotPose(pr2_simple_simulator::SetPose::Request  &,
		    pr2_simple_simulator::SetPose::Response &);

  bool setJointPositions(pr2_simple_simulator::SetJoints::Request  &,
			 pr2_simple_simulator::SetJoints::Response &);

  bool setRobotBaseCommand(pr2_simple_simulator::SetPose::Request  &,
			   pr2_simple_simulator::SetPose::Response &);

  // Service-based controls for recording/replaying trajectories.
  bool beginRecording(pr2_simple_simulator::FilePath::Request  &,
		      pr2_simple_simulator::FilePath::Response &);

  bool endRecording(std_srvs::Empty::Request  &,
		    std_srvs::Empty::Response &);

  bool beginReplay(pr2_simple_simulator::FilePath::Request  &,
		   pr2_simple_simulator::FilePath::Response &);

  bool endReplay(std_srvs::Empty::Request  &,
		 std_srvs::Empty::Response &);

  bool play(std_srvs::Empty::Request  &,
	    std_srvs::Empty::Response &);

  bool pause(std_srvs::Empty::Request  &,
	     std_srvs::Empty::Response &);

  /**
   * @brief Provides a way to notify the simulator of key events recieved
   *        elsewhere.
   */
  bool processKeyEvent(pr2_simple_simulator::KeyEvent::Request  &,
		       pr2_simple_simulator::KeyEvent::Response &);

  bool showBasePath(pr2_simple_simulator::FilePath::Request  &,
		    pr2_simple_simulator::FilePath::Response &);

  void updateEndEffectorMarker();

  void updateEndEffectorMarkerVelocity(const geometry_msgs::Twist &);

  bool isBaseMoving() const;

  bool isValidEndEffectorPose(const geometry_msgs::Pose &);

  void showEndEffectorWorkspaceArc();

private:
  bool playing_;

  void updateTransforms();

  double frame_rate_;

  PViz pviz_;
  interactive_markers::InteractiveMarkerServer int_marker_server_;
  BaseMovementController base_movement_controller_;
  EndEffectorController end_effector_controller_;

  MotionRecorder recorder_;

  ros::Subscriber vel_cmd_sub_;
  ros::Subscriber end_effector_vel_cmd_sub_;
  ros::Subscriber end_effector_marker_vel_sub_;
  
  ros::Publisher base_pose_pub_;
  ros::Publisher joint_states_pub_;
  ros::Publisher end_effector_pose_pub_;
  ros::Publisher marker_pub_;

  geometry_msgs::Twist vel_cmd_;
  geometry_msgs::Twist key_vel_cmd_;
  geometry_msgs::Twist end_effector_vel_cmd_;
  geometry_msgs::Twist end_effector_marker_vel_;
  geometry_msgs::PoseStamped base_pose_;
  geometry_msgs::PoseStamped end_effector_pose_;
  geometry_msgs::PoseStamped end_effector_goal_pose_;
  sensor_msgs::JointState joint_states_;

  ros::ServiceServer set_speed_service_;
  ros::ServiceServer reset_robot_service_;
  ros::ServiceServer set_robot_pose_service_;
  ros::ServiceServer set_joint_states_service_;
  ros::ServiceServer set_base_command_service_;
  ros::ServiceServer begin_rec_service_;
  ros::ServiceServer end_rec_service_;
  ros::ServiceServer begin_replay_service_;
  ros::ServiceServer end_replay_service_;
  ros::ServiceServer pause_service_;
  ros::ServiceServer play_service_;
  ros::ServiceServer key_event_service_;
  ros::ServiceServer show_base_path_service_;

  visualization_msgs::MarkerArray robot_markers_;

  geometry_msgs::PoseStamped goal_pose_;

  sbpl_arm_planner::KDLRobotModel kdl_robot_model_;
  KDL::Frame map_in_torso_lift_link_;
  std::map<std::string, int> joints_map_;

  bool is_moving_r_gripper_;

  tf::TransformBroadcaster tf_broadcaster_;

};

#endif // PR2_SIMPLE_SIMULATOR_H
