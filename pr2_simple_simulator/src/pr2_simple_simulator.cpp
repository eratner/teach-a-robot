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

  // Create a mapping from joint names to index.
  for(int i = 0; i < 14; ++i)
    joints_map_[joint_states_.name[i]] = i;

  vel_cmd_sub_ = nh.subscribe("vel_cmd",
			      100,
			      &PR2SimpleSimulator::updateVelocity,
			      this);

  base_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("base_pose", 20);
  joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 20);

  // Clients can change the speed (linear and angular) at which the robot moves.
  set_speed_service_ = nh.advertiseService("set_speed",
					   &PR2SimpleSimulator::setSpeed,
					   this);
  // Clients can reset the robot (useful for replay).
  reset_robot_service_ = nh.advertiseService("reset_robot",
					     &PR2SimpleSimulator::resetRobot,
					     this);
  // Clients can set the pose of the robot (also useful for replay).
  set_robot_pose_service_ = nh.advertiseService("set_robot_pose",
						&PR2SimpleSimulator::setRobotPose,
						this);
  // Clients can set the state of the joints (also useful for replay).
  set_joint_states_service_ = nh.advertiseService("set_joints",
						  &PR2SimpleSimulator::setJointPositions,
						  this);

  // Attach an interactive marker to the base of the robot.
  updateRobotMarkers();

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/map";
  int_marker.name = "base_marker";
  int_marker.description = "";

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

  // Attach an interactive marker to the end effectors of the robot.
  visualization_msgs::Marker r_ee_marker = robot_markers_.markers.at(11);

  int_marker.header.frame_id = "/map";
  int_marker.pose = r_ee_marker.pose;
  int_marker.name = "r_ee_marker";
  int_marker.description = "";

  r_ee_marker.scale.x = 1.2;
  r_ee_marker.scale.y = 1.2;
  r_ee_marker.scale.z = 1.2;

  control.always_visible = true;
  control.markers.clear();
  control.markers.push_back(r_ee_marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
  int_marker.controls.clear();
  int_marker.controls.push_back(control);
  
  int_marker_server_.insert(int_marker,
			    boost::bind(&PR2SimpleSimulator::gripperMarkerFeedback,
					this,
					_1)
			    );
  int_marker_server_.applyChanges();

  std::string robot_description;
  std::string robot_param = "";
  if(!nh.searchParam("robot_description", robot_param))
  {
    ROS_ERROR("[PR2SimpleSim] Failed to find the robot_description on the parameter server.");
  }
  nh.param<std::string>(robot_param, robot_description, "");
  std::vector<std::string> planning_joints(joint_states_.name.begin()+7, joint_states_.name.end());
  for(std::vector<string>::iterator it = planning_joints.begin();
      it != planning_joints.end();
      ++it)
  {
    ROS_INFO("[PR2SimpleSim] planning joint %s", it->c_str());
  }
  kdl_robot_model_.init(robot_description, planning_joints);
  kdl_robot_model_.setPlanningLink("r_wrist_roll_link");

  ROS_INFO("[PR2SimpleSim] Torso pose: (%f, %f, %f)", robot_markers_.markers.at(1).pose.position.x,
	   robot_markers_.markers.at(1).pose.position.y, robot_markers_.markers.at(1).pose.position.z);

  // Set the map to torso_lift_link transform.
  updateTransforms();
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

    updateTransforms();
  
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

  pviz_.visualizeRobot(r_joints_pos, l_joints_pos, body_pos, 0, 0.3, "simple_sim", 0, true);

  BodyPose body;
  body.x = body_pos[0];
  body.y = body_pos[1];
  body.theta = body_pos[2];
  robot_markers_ = pviz_.getRobotMarkerMsg(r_joints_pos, l_joints_pos, body, 0.3, "simple_sim", 0);

  // Publish the state of the joints as they are being visualized.
  joint_states_pub_.publish(joint_states_);
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

void PR2SimpleSimulator::gripperMarkerFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
)
{
  // Attempt to find joint angles for the arm using the arm IK solver.
  std::vector<double> r_arm_joints(7, 0);
  std::vector<double> end_effector_pose(6, 0);
  ROS_INFO("[PR2SimpleSim] Computing FK with current arm joint angles:");
  for(int i = 7; i < 14; ++i)
  {
    ROS_INFO("%s : %f", joint_states_.name[i].c_str(), joint_states_.position[i]);
    r_arm_joints[i-7] = joint_states_.position[i];
  }
  
  // Find the pose of the end-effector in the map frame.
  if(!kdl_robot_model_.computePlanningLinkFK(r_arm_joints, end_effector_pose))
  {
    ROS_ERROR("[PR2SimpleSim] FK failed!");
  }
  
  ROS_INFO("[PR2SimpleSim] Initial end-effector pose: position (%f, %f, %f) orientation (RPY): (%f, %f, %f)",
	   end_effector_pose[0], end_effector_pose[1], end_effector_pose[2],
	   end_effector_pose[3], end_effector_pose[4], end_effector_pose[5]);

  std::vector<double> goal_end_effector_pose(7, 0);
  goal_end_effector_pose[0] = feedback->pose.position.x;
  goal_end_effector_pose[1] = feedback->pose.position.y;
  goal_end_effector_pose[2] = feedback->pose.position.z;
  goal_end_effector_pose[3] = feedback->pose.orientation.x;
  goal_end_effector_pose[4] = feedback->pose.orientation.y;
  goal_end_effector_pose[5] = feedback->pose.orientation.z;
  goal_end_effector_pose[6] = feedback->pose.orientation.w;

  ROS_INFO("[PR2SimpleSim] Goal end-effector pose: position (%f, %f, %f) orientation: (%f, %f, %f, %f)",
	   goal_end_effector_pose[0], goal_end_effector_pose[1], goal_end_effector_pose[2],
	   goal_end_effector_pose[3], goal_end_effector_pose[4], goal_end_effector_pose[5],
	   goal_end_effector_pose[6]);

  // Use IK to find the required joint angles for the arm.
  std::vector<double> solution(7, 0);
  if(!kdl_robot_model_.computeIK(goal_end_effector_pose, r_arm_joints, solution))
  {
    ROS_ERROR("[PR2SimpleSim] IK failed!");
  }
  else
  {
    // Set the new joint angles.
    for(int i = 7; i < joint_states_.position.size(); ++i)
      joint_states_.position[i] = solution[i-7];

    ROS_INFO("[PR2SimpleSim] IK found the following joint angles for this end-effector pose:");
    std::vector<double>::iterator it;
    for(it = solution.begin(); it != solution.end(); ++it)
      ROS_INFO("%f", *it);
  }
}

bool PR2SimpleSimulator::setSpeed(pr2_simple_simulator::SetSpeed::Request  &req,
				     pr2_simple_simulator::SetSpeed::Response &res)
{
  if(req.linear > 0)
  {
    ROS_INFO("[PR2SimpleSim] Setting linear speed to %f.", req.linear);
    base_movement_controller_.setLinearSpeed(req.linear);
  }

  if(req.angular > 0)
  {
    ROS_INFO("[PR2SimpleSim] Setting angular speed to %f.", req.angular);
    base_movement_controller_.setAngularSpeed(req.angular);
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

bool PR2SimpleSimulator::setRobotPose(pr2_simple_simulator::SetPose::Request  &req,
				      pr2_simple_simulator::SetPose::Response &res)
{
  base_pose_ = req.pose;

  return true;
}

bool PR2SimpleSimulator::setJointPositions(pr2_simple_simulator::SetJoints::Request  &req,
					   pr2_simple_simulator::SetJoints::Response &res)
{
  ROS_ASSERT(req.name.size() == req.position.size());

  for(int i = 0; i < req.name.size(); ++i)
  {
    if(joints_map_.find(req.name[i]) != joints_map_.end())
    {
      joint_states_.position[joints_map_[req.name[i]]] = req.position[i];
    }
    else
      ROS_ERROR("[PR2SimpleSim] Failed to find index for joint \"%s\"!", req.name[i].c_str());
  }

  return true;
}

void PR2SimpleSimulator::updateTransforms()
{
  // @todo clean this up and figure out better names!!
  // Get the map -> base_footprint transform.
  KDL::Frame map_to_base_footprint;
  map_to_base_footprint.p.x(base_pose_.pose.position.x);
  map_to_base_footprint.p.y(base_pose_.pose.position.y);
  map_to_base_footprint.p.z(base_pose_.pose.position.z);
  map_to_base_footprint.M = KDL::Rotation::Quaternion(base_pose_.pose.orientation.x,
						      base_pose_.pose.orientation.y,
						      base_pose_.pose.orientation.z,
						      base_pose_.pose.orientation.w);
  map_to_base_footprint = map_to_base_footprint.Inverse();

  // Get the base_footprint -> torso_lift_link transform (hack, from gazebo).
  KDL::Frame base_to_torso_lift_link;
  base_to_torso_lift_link.p.x(0.050);
  base_to_torso_lift_link.p.y(0.0);
  base_to_torso_lift_link.p.z(-0.803); // @todo if raising the robot, add to this offest.
  base_to_torso_lift_link.M = KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

  map_to_torso_lift_link_ = base_to_torso_lift_link * map_to_base_footprint;

  double r, p, y;
  map_to_torso_lift_link_.M.GetRPY(r, p, y);
  // ROS_INFO("[PR2SimpleSim] map -> torso_lift_link: translation: (%f, %f, %f); rotation (RPY): (%f, %f, %f)",
  // 	   map_to_torso_lift_link_.p.x(), map_to_torso_lift_link_.p.y(), map_to_torso_lift_link_.p.z(),
  // 	   r, p, y);

  kdl_robot_model_.setKinematicsToPlanningTransform(map_to_torso_lift_link_.Inverse(), "map");
}
