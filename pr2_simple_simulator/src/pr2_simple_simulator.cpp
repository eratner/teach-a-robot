#include "pr2_simple_simulator/pr2_simple_simulator.h"

PR2SimpleSimulator::PR2SimpleSimulator()
  : pviz_(), int_marker_server_("simple_sim_marker"), base_movement_controller_()
{
  ros::NodeHandle nh;

  // Initialize the velocity command (initially at rest).
  vel_cmd_.linear.x = vel_cmd_.linear.y = vel_cmd_.linear.z = 0;
  vel_cmd_.angular.x = vel_cmd_.angular.y = vel_cmd_.angular.z = 0;

  // Initialize the base pose.
  base_pose_.header.stamp = ros::Time();
  base_pose_.header.frame_id = "/map";
  base_pose_.pose.position.x = 0;
  base_pose_.pose.position.y = 0;
  base_pose_.pose.position.z = 0;
  base_pose_.pose.orientation.x = 0;
  base_pose_.pose.orientation.y = 0;
  base_pose_.pose.orientation.z = 0;
  base_pose_.pose.orientation.w = 1;

  // Initialize the joint states.
  joint_states_.name.resize(14);
  joint_states_.name[0] = "l_shoulder_pan_joint";
  joint_states_.name[1] = "l_shoulder_lift_joint";
  joint_states_.name[2] = "l_upper_arm_roll_joint";
  joint_states_.name[3] = "l_elbow_flex_joint";
  joint_states_.name[4] = "l_forearm_roll_joint";
  joint_states_.name[5] = "l_wrist_flex_joint";
  joint_states_.name[6] = "l_wrist_roll_joint";
  joint_states_.name[7] = "r_shoulder_pan_joint";
  joint_states_.name[8] = "r_shoulder_lift_joint";
  joint_states_.name[9] = "r_upper_arm_roll_joint";
  joint_states_.name[10] = "r_elbow_flex_joint";
  joint_states_.name[11] = "r_forearm_roll_joint";
  joint_states_.name[12] = "r_wrist_flex_joint";
  joint_states_.name[13] = "r_wrist_roll_joint";
  joint_states_.position.resize(14);
  joint_states_.velocity.resize(14);
  joint_states_.effort.resize(14);
  for(int i = 0; i < 14; ++i)
    joint_states_.position[i] = joint_states_.velocity[i] = joint_states_.effort[i] = 0;

  vel_cmd_sub_ = nh.subscribe("vel_cmd",
			      100,
			      &PR2SimpleSimulator::updateVelocity,
			      this);

  base_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("base_pose", 20);
  joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 20);

  // Users can change the velocity (linear and angular) at which the robot moves.
  set_vel_service_ = nh.advertiseService("set_vel",
					 &PR2SimpleSimulator::setVelocity,
					 this);
  // Users can reset the robot (useful for replay).
  reset_robot_service_ = nh.advertiseService("reset_robot",
					     &PR2SimpleSimulator::resetRobot,
					     this);

  // Attach an interactive marker to the base of the robot.
  updateRobotMarkers();

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/map";
  int_marker.name = "base_marker";
  int_marker.description = "Base Marker";

  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  // The base marker should be slightly bigger than the actual marker.
  visualization_msgs::Marker base_marker = robot_markers_.markers.at(0);
  base_marker.scale.x = 1.2;
  base_marker.scale.y = 1.2;
  base_marker.scale.z = 1.2;
  control.markers.push_back(base_marker);
  control.always_visible = true;
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  int_marker_server_.insert(int_marker,
			    boost::bind(&PR2SimpleSimulator::baseMarkerFeedback,
					this,
					_1)
			    );
  int_marker_server_.applyChanges();
}

PR2SimpleSimulator::~PR2SimpleSimulator()
{

}

void PR2SimpleSimulator::run()
{
  ros::Rate loop(10.0);
  while(ros::ok())
  {
    moveRobot();

    updateRobotMarkers();
  
    ros::spinOnce();
    loop.sleep();
  }
}

void PR2SimpleSimulator::updateVelocity(const geometry_msgs::Twist &vel)
{
  vel_cmd_ = vel;
}

void PR2SimpleSimulator::moveRobot()
{
  if(base_movement_controller_.getState() == BaseMovementController::DONE)
  {
    int_marker_server_.setPose("base_marker", base_pose_.pose);
    int_marker_server_.applyChanges();
    base_movement_controller_.setState(BaseMovementController::INITIAL);
    return;
  }

  updateVelocity(base_movement_controller_.getNextVelocities(base_pose_.pose, goal_pose_.pose));

  // Get the next velocity commands, and apply them to the robot.
  double theta = tf::getYaw(base_pose_.pose.orientation);
  base_pose_.pose.position.x += (0.1)*vel_cmd_.linear.x*std::cos(theta) 
    - (0.1)*vel_cmd_.linear.y*std::sin(theta);
  base_pose_.pose.position.y += (0.1)*vel_cmd_.linear.y*std::cos(theta)
    + (0.1)*vel_cmd_.linear.x*std::sin(theta);

  base_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(
				  tf::getYaw(base_pose_.pose.orientation) + (0.1)*vel_cmd_.angular.z
				);

  base_pose_pub_.publish(base_pose_);
}

void PR2SimpleSimulator::updateRobotMarkers()
{
  std::vector<double> l_joints_pos(7, 0);
  std::vector<double> r_joints_pos(7, 0);
  for(int i = 0; i < 7; ++i)
  {
    l_joints_pos[i] = joint_states_.position.at(i);
    r_joints_pos[i] = joint_states_.position.at(i+7);
  }

  std::vector<double> body_pos(3, 0);
  body_pos[0] = base_pose_.pose.position.x;
  body_pos[1] = base_pose_.pose.position.y;
  body_pos[2] = tf::getYaw(base_pose_.pose.orientation);

  pviz_.visualizeRobot(l_joints_pos, r_joints_pos, body_pos, 0, 0.3, "simple_sim", 0);

  BodyPose body;
  body.x = body_pos[0];
  body.y = body_pos[1];
  body.theta = body_pos[2];
  robot_markers_ = pviz_.getRobotMarkerMsg(l_joints_pos, r_joints_pos, body, 0.3, "simple_sim", 0);
}

void PR2SimpleSimulator::baseMarkerFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
)
{
  switch(feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    break;
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    {
      base_movement_controller_.setState(BaseMovementController::READY);
      goal_pose_.pose = feedback->pose;
      double yaw = std::atan2(goal_pose_.pose.position.y - base_pose_.pose.position.y,
			      goal_pose_.pose.position.x - base_pose_.pose.position.x);
      ROS_INFO("[PR2SimpleSim] New goal set at (x, y, yaw) = (%f, %f, %f).", 
      	       goal_pose_.pose.position.x,
      	       goal_pose_.pose.position.y,
      	       yaw * (180.0/M_PI));

      break;
    }
  default:
    break;
  }
}

bool PR2SimpleSimulator::setVelocity(pr2_simple_simulator::SetVelocity::Request  &req,
				     pr2_simple_simulator::SetVelocity::Response &res)
{
  if(req.linear > 0)
  {
    ROS_INFO("[PR2SimpleSim] Setting linear velocity to %f.", req.linear);
    base_movement_controller_.setLinearVelocity(req.linear);
  }

  if(req.angular > 0)
  {
    ROS_INFO("[PR2SimpleSim] Setting angular velocity to %f.", req.angular);
    base_movement_controller_.setAngularVelocity(req.angular);
  }

  return true;
}

bool PR2SimpleSimulator::resetRobot(std_srvs::Empty::Request  &req,
				    std_srvs::Empty::Response &res)
{
  ROS_INFO("[PR2SimpleSim] Resetting robot...");

  // Reset the base pose.
  base_pose_.pose.position.x = 0;
  base_pose_.pose.position.y = 0;
  base_pose_.pose.position.z = 0;
  base_pose_.pose.orientation.x = 0;
  base_pose_.pose.orientation.y = 0;
  base_pose_.pose.orientation.z = 0;
  base_pose_.pose.orientation.w = 1;

  // Reset the goal pose.
  goal_pose_ = base_pose_;
  
  // Reset the base pose interactive marker.
  int_marker_server_.setPose("base_marker", base_pose_.pose);
  int_marker_server_.applyChanges();

  // Reset the joints.
  for(int i = 0; i < joint_states_.position.size(); ++i)
    joint_states_.position[i] = 0;

  // Reset the base movement controller.
  base_movement_controller_.setState(BaseMovementController::INITIAL);

  return true;
}
