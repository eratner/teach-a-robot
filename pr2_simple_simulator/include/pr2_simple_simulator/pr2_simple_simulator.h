#ifndef PR2_SIMPLE_SIMULATOR_H
#define PR2_SIMPLE_SIMULATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <pviz/pviz.h>
#include <interactive_markers/interactive_marker_server.h>
#include "pr2_simple_simulator/base_movement_controller.h"

#include <vector>
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

private:
  PViz pviz_;
  interactive_markers::InteractiveMarkerServer int_marker_server_;
  BaseMovementController base_movement_controller_;

  ros::Subscriber vel_cmd_sub_;
  
  ros::Publisher base_pose_pub_;
  ros::Publisher joint_states_pub_;

  geometry_msgs::Twist vel_cmd_;
  geometry_msgs::PoseStamped base_pose_;
  sensor_msgs::JointState joint_states_;

  visualization_msgs::MarkerArray robot_markers_;

  geometry_msgs::PoseStamped goal_pose_;
  bool has_new_goal_;

};

#endif // PR2_SIMPLE_SIMULATOR_H
