#ifndef PR2_SIMPLE_SIMULATOR_H
#define PR2_SIMPLE_SIMULATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <pviz/pviz.h>
#include <interactive_markers/interactive_marker_server.h>
#include <pr2_simple_simulator/base_movement_controller.h>
#include <pr2_simple_simulator/SetSpeed.h>
#include <pr2_simple_simulator/SetPose.h>
#include <sbpl_manipulation_components/kdl_robot_model.h>

#include <vector>
#include <string>
#include <cmath>

class PR2SimpleSimulator
{
public:
  PR2SimpleSimulator();

  ~PR2SimpleSimulator();

  void run();

  void updateVelocity(const geometry_msgs::Twist &);

  void moveRobot();

  void updateRobotMarkers();

  void baseMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void gripperMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  bool setSpeed(pr2_simple_simulator::SetSpeed::Request  &,
		   pr2_simple_simulator::SetSpeed::Response &);

  bool resetRobot(std_srvs::Empty::Request  &,
		  std_srvs::Empty::Response &);

  bool setRobotPose(pr2_simple_simulator::SetPose::Request  &,
		    pr2_simple_simulator::SetPose::Response &);

  // @todo
  // bool setJointPositions(...);

private:
  void updateTransforms();

  PViz pviz_;
  interactive_markers::InteractiveMarkerServer int_marker_server_;
  BaseMovementController base_movement_controller_;

  ros::Subscriber vel_cmd_sub_;
  
  ros::Publisher base_pose_pub_;
  ros::Publisher joint_states_pub_;

  geometry_msgs::Twist vel_cmd_;
  geometry_msgs::PoseStamped base_pose_;
  sensor_msgs::JointState joint_states_;

  ros::ServiceServer set_speed_service_;
  ros::ServiceServer reset_robot_service_;
  ros::ServiceServer set_robot_pose_service_;

  visualization_msgs::MarkerArray robot_markers_;

  geometry_msgs::PoseStamped goal_pose_;

  sbpl_arm_planner::KDLRobotModel kdl_robot_model_;
  KDL::Frame map_to_torso_lift_link_;

};

#endif // PR2_SIMPLE_SIMULATOR_H
