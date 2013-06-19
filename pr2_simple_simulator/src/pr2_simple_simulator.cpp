#include "pr2_simple_simulator/pr2_simple_simulator.h"

PR2SimpleSimulator::PR2SimpleSimulator()
  : frame_rate_(10.0), pviz_(), int_marker_server_("simple_sim_marker"), base_movement_controller_()
{
  ros::NodeHandle nh;

  // Initialize the velocity command (initially at rest).
  vel_cmd_.linear.x = vel_cmd_.linear.y = vel_cmd_.linear.z = 0;
  vel_cmd_.angular.x = vel_cmd_.angular.y = vel_cmd_.angular.z = 0;

  // Initialize the velocity command for the end-effector (initially at rest).
  end_effector_vel_cmd_.linear.x = end_effector_vel_cmd_.linear.y = end_effector_vel_cmd_.linear.z = 0;
  end_effector_vel_cmd_.angular.x = end_effector_vel_cmd_.angular.y = end_effector_vel_cmd_.angular.z = 0;

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

  goal_pose_ = base_pose_;

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

  end_effector_vel_cmd_sub_ = nh.subscribe("end_effector_vel_cmd",
					   100,
					   &PR2SimpleSimulator::updateEndEffectorVelocity,
					   this);

  base_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("base_pose", 20);
  joint_states_pub_ = nh.advertise<sensor_msgs::JointState>("joint_states", 20);
  end_effector_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>("end_effector_pose", 20);
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

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

  begin_rec_service_ = nh.advertiseService("/motion_recorder/begin_recording", 
					   &PR2SimpleSimulator::beginRecording,
					   this);

  end_rec_service_ = nh.advertiseService("/motion_recorder/end_recording",
					 &PR2SimpleSimulator::endRecording,
					 this);

  begin_replay_service_ = nh.advertiseService("/motion_recorder/begin_replay",
					      &PR2SimpleSimulator::beginReplay,
					      this);

  end_replay_service_ = nh.advertiseService("/motion_recorder/end_replay",
					    &PR2SimpleSimulator::endReplay,
					    this);

  // Attach an interactive marker to the base of the robot.
  visualizeRobot();
  
  // Initialize the end-effector pose.
  end_effector_pose_.header.frame_id = "/map";
  end_effector_pose_.header.stamp = ros::Time();
  end_effector_pose_.pose = robot_markers_.markers.at(11).pose;

  end_effector_goal_pose_ = end_effector_pose_;

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
  base_marker.color.r = 0;
  base_marker.color.g = 1;
  base_marker.color.b = 0;
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
  visualization_msgs::Marker r_gripper_marker = robot_markers_.markers.at(11);

  int_marker.header.frame_id = "/map";
  int_marker.pose = r_gripper_marker.pose;
  int_marker.name = "r_gripper_marker";
  int_marker.description = "";

  r_gripper_marker.scale.x = 1.2;
  r_gripper_marker.scale.y = 1.2;
  r_gripper_marker.scale.z = 1.2;
  r_gripper_marker.color.r = 0;
  r_gripper_marker.color.g = 1;
  r_gripper_marker.color.b = 0;

  control.always_visible = true;
  control.markers.clear();
  control.markers.push_back(r_gripper_marker);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
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
  kdl_robot_model_.init(robot_description, planning_joints);
  kdl_robot_model_.setPlanningLink("r_wrist_roll_link");

  // Set the map to torso_lift_link transform.
  updateTransforms();

  is_moving_r_gripper_ = false;
}

PR2SimpleSimulator::~PR2SimpleSimulator()
{

}

void PR2SimpleSimulator::setFrameRate(double rate)
{
  frame_rate_ = rate;
}

double PR2SimpleSimulator::getFrameRate() const
{
  return frame_rate_;
}

void PR2SimpleSimulator::run()
{
  ros::Rate loop(getFrameRate());
  while(ros::ok())
  {
    // Record motion.
    if(recorder_.isRecording())
    {
      recorder_.recordBasePose(base_pose_);
      recorder_.recordJoints(joint_states_);
    }

    visualizeRobot();

    updateTransforms();

    // Replay motion.
    if(recorder_.isReplaying())
    {
      if(recorder_.getJointsRemaining() == 0 && recorder_.getPosesRemaining() == 0)
      {
	recorder_.endReplay();
	marker_pub_.publish(recorder_.getBasePath());
	ROS_INFO("[PR2SimpleSim] Done replaying.");
      }

      if(recorder_.getJointsRemaining() > 0)
	joint_states_ = recorder_.getNextJoints();

      if(recorder_.getPosesRemaining() > 0)
      {
	base_pose_ = recorder_.getNextBasePose();

	int_marker_server_.setPose("base_marker", base_pose_.pose);
	int_marker_server_.applyChanges();
      }
    }
    else
    {
      moveEndEffectors();

      moveRobot();
    }
  
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
  base_pose_.pose.position.x += (1.0/getFrameRate())*vel_cmd_.linear.x*std::cos(theta) 
    - (1.0/getFrameRate())*vel_cmd_.linear.y*std::sin(theta);
  base_pose_.pose.position.y += (1.0/getFrameRate())*vel_cmd_.linear.y*std::cos(theta)
    + (1.0/getFrameRate())*vel_cmd_.linear.x*std::sin(theta);

  base_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(
				  tf::getYaw(base_pose_.pose.orientation) 
				  + (1/getFrameRate())*vel_cmd_.angular.z
				);

  base_pose_pub_.publish(base_pose_);
}

void PR2SimpleSimulator::moveEndEffectors()
{
  geometry_msgs::Twist vel = end_effector_controller_.moveTo(end_effector_pose_.pose,
							     end_effector_goal_pose_.pose);

  end_effector_vel_cmd_.linear.x = vel.linear.x;
  end_effector_vel_cmd_.linear.y = vel.linear.y;

  if(end_effector_vel_cmd_.linear.x == 0 &&
     end_effector_vel_cmd_.linear.y == 0 &&
     end_effector_vel_cmd_.linear.z == 0)
    return;

  double theta = tf::getYaw(end_effector_pose_.pose.orientation);//-tf::getYaw(base_pose_.pose.orientation);

  // Apply the next end-effector velocity commands (in the robot's base frame.)
  geometry_msgs::Pose next_end_effector_pose = end_effector_pose_.pose;
  next_end_effector_pose.position.x += (1/getFrameRate())*end_effector_vel_cmd_.linear.x*std::cos(theta)
    - (1/getFrameRate())*end_effector_vel_cmd_.linear.y*std::sin(theta);
  next_end_effector_pose.position.y += (1/getFrameRate())*end_effector_vel_cmd_.linear.y*std::cos(theta)
    + (1/getFrameRate())*end_effector_vel_cmd_.linear.x*std::sin(theta);
  next_end_effector_pose.position.z += (1/getFrameRate())*end_effector_vel_cmd_.linear.z;
  
  if(!setEndEffectorPose(next_end_effector_pose))
  {
    end_effector_controller_.setState(EndEffectorController::INVALID_GOAL);
  }
}

void PR2SimpleSimulator::visualizeRobot()
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

  if(!is_moving_r_gripper_ && end_effector_controller_.getState() == EndEffectorController::DONE)
  {
    int_marker_server_.setPose("r_gripper_marker", robot_markers_.markers.at(11).pose);
    int_marker_server_.applyChanges();
  }

  // Publish the pose of the right end effector.
  end_effector_pose_.pose = robot_markers_.markers.at(11).pose;
  end_effector_pose_pub_.publish(end_effector_pose_);

  // Publish the state of the joints as they are being visualized.
  joint_states_pub_.publish(joint_states_);
}

bool PR2SimpleSimulator::setEndEffectorPose(const geometry_msgs::Pose &goal_pose)
{
  // Attempt to find joint angles for the arm using the arm IK solver.
  std::vector<double> r_arm_joints(7, 0);
  for(int i = 7; i < 14; ++i)
  {
    r_arm_joints[i-7] = joint_states_.position[i];
  }

  std::vector<double> goal_end_effector_pose(7, 0);
  goal_end_effector_pose[0] = goal_pose.position.x;
  goal_end_effector_pose[1] = goal_pose.position.y;
  goal_end_effector_pose[2] = goal_pose.position.z;
  goal_end_effector_pose[3] = goal_pose.orientation.x;
  goal_end_effector_pose[4] = goal_pose.orientation.y;
  goal_end_effector_pose[5] = goal_pose.orientation.z;
  goal_end_effector_pose[6] = goal_pose.orientation.w;
  
  // Use IK to find the required joint angles for the arm.
  std::vector<double> solution(7, 0);
  if(!kdl_robot_model_.computeIK(goal_end_effector_pose, r_arm_joints, solution))
  {
    return false;
  }

  std::vector<double> fk_pose;
  if(!kdl_robot_model_.computeFK(solution, "r_wrist_roll_joint", fk_pose))
  {
    ROS_ERROR("Failed to compute FK!");
  }
  else
  {
    ROS_INFO("FK pose: (%f, %f, %f), (%f, %f, %f), difference in z: %f",
	     fk_pose[0], fk_pose[1], fk_pose[2],  fk_pose[3], fk_pose[4], fk_pose[5],
	     fk_pose[0] - goal_pose.position.z);
  }

  // Set the new joint angles.
  for(int i = 7; i < joint_states_.position.size(); ++i)
    joint_states_.position[i] = solution[i-7];
  
  end_effector_pose_.pose = goal_pose;

  return true;
}

void PR2SimpleSimulator::updateEndEffectorVelocity(const geometry_msgs::Twist &vel)
{
  end_effector_vel_cmd_ = vel;
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
  switch(feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    is_moving_r_gripper_ = true;
    break;
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    {
      end_effector_controller_.setState(EndEffectorController::READY);
      is_moving_r_gripper_ = false;
      ROS_INFO("[PR2SimpleSim] Setting new end effector goal position at (%f, %f, %f).",
	       feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      end_effector_goal_pose_.pose = feedback->pose;
      break;
    }
  default:
    break;
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

  // Reset the end effector controller.
  end_effector_controller_.setState(EndEffectorController::DONE);

  // Delete the drawn base path.
  visualization_msgs::Marker base_path;
  base_path.header.frame_id = "/map";
  base_path.ns = "motion_rec";
  base_path.id = 0;
  base_path.action = visualization_msgs::Marker::DELETE;
  base_path.type = visualization_msgs::Marker::LINE_STRIP;
  marker_pub_.publish(base_path);

  return true;
}

bool PR2SimpleSimulator::setRobotPose(pr2_simple_simulator::SetPose::Request  &req,
				      pr2_simple_simulator::SetPose::Response &res)
{
  base_pose_ = req.pose;

  if(base_movement_controller_.getState() == BaseMovementController::INITIAL ||
     base_movement_controller_.getState() == BaseMovementController::DONE)
  {
    int_marker_server_.setPose("base_marker", base_pose_.pose);
    int_marker_server_.applyChanges();
  }

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

bool PR2SimpleSimulator::beginRecording(pr2_simple_simulator::FilePath::Request  &req,
					pr2_simple_simulator::FilePath::Response &res)
{
  recorder_.beginRecording(req.file_path);

  return true;
}

bool PR2SimpleSimulator::endRecording(std_srvs::Empty::Request  &,
				      std_srvs::Empty::Response &)
{
  recorder_.endRecording();

  return true;
}

bool PR2SimpleSimulator::beginReplay(pr2_simple_simulator::FilePath::Request  &req,
				     pr2_simple_simulator::FilePath::Response &res)
{
  std_srvs::Empty empty;
  if(!resetRobot(empty.request, empty.response))
    return false;

  recorder_.beginReplay(req.file_path);

  return true;
}

bool PR2SimpleSimulator::endReplay(std_srvs::Empty::Request  &,
				   std_srvs::Empty::Response &)
{
  recorder_.endReplay();

  return true;
}

void PR2SimpleSimulator::updateTransforms()
{
  // Broadcast the coordinate frame of the base footprint.
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(base_pose_.pose.position.x,
				  base_pose_.pose.position.y,
				  base_pose_.pose.position.z)
		      );
  transform.setRotation(tf::Quaternion(base_pose_.pose.orientation.x,
				       base_pose_.pose.orientation.y,
				       base_pose_.pose.orientation.z,
				       base_pose_.pose.orientation.w)
			);
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform,
						     ros::Time::now(),
						     "map",
						     "base_footprint")
				);

  // Get the pose of the base footprint frame with respect to the map frame.
  KDL::Frame base_footprint_in_map;
  base_footprint_in_map.p.x(base_pose_.pose.position.x);
  base_footprint_in_map.p.y(base_pose_.pose.position.y);
  base_footprint_in_map.p.z(base_pose_.pose.position.z);
  base_footprint_in_map.M = KDL::Rotation::Quaternion(base_pose_.pose.orientation.x,
   						      base_pose_.pose.orientation.y,
  						      base_pose_.pose.orientation.z,
						      base_pose_.pose.orientation.w);

  // Get the pose of the base footprint in the torso lift link frame.
  KDL::Frame base_in_torso_lift_link;
  base_in_torso_lift_link.p.x(0.050);
  base_in_torso_lift_link.p.y(0.0);
  base_in_torso_lift_link.p.z(-0.803);
  base_in_torso_lift_link.M = KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

  map_in_torso_lift_link_ = base_footprint_in_map.Inverse() * base_in_torso_lift_link;

  // double r, p, y;
  // map_in_torso_lift_link_.Inverse().M.GetRPY(r, p, y);
  // ROS_INFO("kinematics to planning origin: (%f, %f, %f), rotation: (%f, %f, %f)",
  // 	   map_in_torso_lift_link_.Inverse().p.x(), map_in_torso_lift_link_.Inverse().p.y(),
  // 	   map_in_torso_lift_link_.Inverse().p.z(), r, p, y);

  kdl_robot_model_.setKinematicsToPlanningTransform(map_in_torso_lift_link_.Inverse(), "map");
}
