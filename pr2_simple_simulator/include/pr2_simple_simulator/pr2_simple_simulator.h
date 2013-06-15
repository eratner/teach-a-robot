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
#include <pr2_simple_simulator/SetSpeed.h>
#include <pr2_simple_simulator/SetPose.h>
#include <pr2_simple_simulator/SetJoints.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>

#include <vector>
#include <string>
#include <cmath>
#include <map>

class PR2SimpleSimulator
{
public:
  PR2SimpleSimulator();

  ~PR2SimpleSimulator();

  void run();

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
   * @brief Finds and sets the appropriate positions of the
   *        (right) arm joints using IK so that the end-effector 
   *        is at the desired pose.
   * @param the desired pose of the (right) end-effector.
   * @return true on success, otherwise false.
   */
  bool setEndEffectorPose(const geometry_msgs::Pose &);

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

private:
  void updateTransforms();

  PViz pviz_;
  interactive_markers::InteractiveMarkerServer int_marker_server_;
  BaseMovementController base_movement_controller_;

  ros::Subscriber vel_cmd_sub_;
  ros::Subscriber end_effector_vel_cmd_sub_;
  
  ros::Publisher base_pose_pub_;
  ros::Publisher joint_states_pub_;
  ros::Publisher end_effector_pose_pub_;

  geometry_msgs::Twist vel_cmd_;
  geometry_msgs::Twist end_effector_vel_cmd_;
  geometry_msgs::PoseStamped base_pose_;
  geometry_msgs::PoseStamped end_effector_pose_;
  sensor_msgs::JointState joint_states_;

  ros::ServiceServer set_speed_service_;
  ros::ServiceServer reset_robot_service_;
  ros::ServiceServer set_robot_pose_service_;
  ros::ServiceServer set_joint_states_service_;

  visualization_msgs::MarkerArray robot_markers_;

  geometry_msgs::PoseStamped goal_pose_;

  sbpl_arm_planner::KDLRobotModel kdl_robot_model_;
  KDL::Frame map_to_torso_lift_link_;
  std::map<std::string, int> joints_map_;

  bool is_moving_r_gripper_;

};

#endif // PR2_SIMPLE_SIMULATOR_H
