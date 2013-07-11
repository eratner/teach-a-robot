#include "demonstration_visualizer/pr2_simulator.h"

namespace demonstration_visualizer {

PR2Simulator::PR2Simulator(MotionRecorder *recorder, 
			   PViz *pviz,
			   interactive_markers::InteractiveMarkerServer *int_marker_server,
         ObjectManager* object_manager)
  : playing_(true), 
    attached_object_(false),
    frame_rate_(10.0), 
    pviz_(pviz), 
    int_marker_server_(int_marker_server), 
    object_manager_(object_manager),
    base_movement_controller_(),
    end_effector_controller_(),
    recorder_(recorder)
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
  joint_states_.name.resize(15);
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
  joint_states_.name[14] = "r_gripper_joint";
  joint_states_.position.resize(15);
  joint_states_.velocity.resize(15);
  joint_states_.effort.resize(15);
  // Tuck the left arm initially. @todo these should be constants somewhere.
  joint_states_.position[0] = 0.06024;
  joint_states_.position[1] = 1.248526;
  joint_states_.position[2] = 1.789070;
  joint_states_.position[3] = -1.683386;
  joint_states_.position[4] = -1.7343417;
  joint_states_.position[5] = -0.0962141;
  joint_states_.position[6] = -0.0864407;

  for(int i = 7; i < 15; ++i)
    joint_states_.position[i] = joint_states_.velocity[i] = joint_states_.effort[i] = 0;

  // Create a mapping from joint names to index.
  for(int i = 0; i < 15; ++i)
    joints_map_[joint_states_.name[i]] = i;

  vel_cmd_sub_ = nh.subscribe("vel_cmd",
			      100,
			      &PR2Simulator::updateVelocity,
			      this);

  end_effector_vel_cmd_sub_ = nh.subscribe("end_effector_vel_cmd",
					   100,
					   &PR2Simulator::updateEndEffectorVelocity,
					   this);

  end_effector_marker_vel_sub_ = nh.subscribe("end_effector_marker_vel",
					      1,
					      &PR2Simulator::updateEndEffectorMarkerVelocity,
					      this);

  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1000);

  // Attach an interactive marker to the base of the robot.
  visualizeRobot();
  
  // Initialize the end-effector pose.
  end_effector_pose_.header.frame_id = "/base_footprint";
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

  int_marker_server_->insert(int_marker,
			     boost::bind(&PR2Simulator::baseMarkerFeedback,
					 this,
					 _1)
			     );
  int_marker_server_->applyChanges();

  // Attach an interactive marker to the end effectors of the robot.
  visualization_msgs::Marker r_gripper_marker = robot_markers_.markers.at(11);

  //int_marker.header.frame_id = "/map";
  int_marker.header.frame_id = "/base_footprint";
  int_marker.pose = robot_markers_.markers.at(11).pose;//r_gripper_marker.pose;
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

  control.markers[0].pose = geometry_msgs::Pose();
  control.markers[0].header = std_msgs::Header();

  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  int_marker.controls.clear();
  int_marker.controls.push_back(control);
  
  int_marker_server_->insert(int_marker,
			     boost::bind(&PR2Simulator::gripperMarkerFeedback,
					 this,
					 _1)
			     );
  int_marker_server_->applyChanges();

  std::string robot_description;
  std::string robot_param = "";
  if(!nh.searchParam("robot_description", robot_param))
  {
    ROS_ERROR("[PR2SimpleSim] Failed to find the robot_description on the parameter server.");
  }
  nh.param<std::string>(robot_param, robot_description, "");
  std::vector<std::string> planning_joints(joint_states_.name.begin()+7, joint_states_.name.end()-1);
  kdl_robot_model_.init(robot_description, planning_joints);
  kdl_robot_model_.setPlanningLink("r_wrist_roll_link");

  // Set the map to torso_lift_link transform.
  updateTransforms();

  is_moving_r_gripper_ = false;
}

PR2Simulator::~PR2Simulator()
{

}

void PR2Simulator::setFrameRate(double rate)
{
  frame_rate_ = rate;
}

double PR2Simulator::getFrameRate() const
{
  return frame_rate_;
}

void PR2Simulator::play()
{
  playing_ = true;
}

void PR2Simulator::pause()
{
  playing_ = false;

  // Set all velocity commands to zero.
  key_vel_cmd_.linear.x = 0;
  key_vel_cmd_.linear.y = 0;
  key_vel_cmd_.linear.z = 0;

  end_effector_marker_vel_.linear.x = 0;
  end_effector_marker_vel_.linear.y = 0;
  end_effector_marker_vel_.linear.z = 0;
}

bool PR2Simulator::isPlaying() const
{
  return playing_;
}

void PR2Simulator::run()
{
  if(isPlaying())
  {
    if(!recorder_->isReplaying())
      moveRobot();

    visualizeRobot();

    updateTransforms();

    updateEndEffectorPose();

    updateEndEffectorMarker();

    // Record motion.
    if(recorder_->isRecording())
    {
      recorder_->recordBasePose(base_pose_);
      recorder_->recordJoints(joint_states_);
    }

    showEndEffectorWorkspaceArc();

    // Replay motion.
    if(recorder_->isReplaying() && isPlaying())
    {
      if(recorder_->getJointsRemaining() == 0 && recorder_->getPosesRemaining() == 0)
      {
	recorder_->endReplay();
	marker_pub_.publish(recorder_->getBasePath());
	ROS_INFO("[PR2Sim] Done replaying.");
      }

      if(recorder_->getJointsRemaining() > 0)
	joint_states_ = recorder_->getNextJoints();

      if(recorder_->getPosesRemaining() > 0)
      {
	base_pose_ = recorder_->getNextBasePose();

	int_marker_server_->setPose("base_marker", base_pose_.pose);
	int_marker_server_->applyChanges();
      }
    }
  }
}

void PR2Simulator::moveRobot()
{
  // First, get the next proposed base pose.
  geometry_msgs::Twist base_vel = base_movement_controller_.getNextVelocities(base_pose_.pose,
									      goal_pose_.pose);
  base_vel.linear.x += vel_cmd_.linear.x;
  base_vel.linear.x += key_vel_cmd_.linear.x;
  base_vel.linear.y += vel_cmd_.linear.y;
  base_vel.linear.y += key_vel_cmd_.linear.y;
  base_vel.angular.z += vel_cmd_.angular.z;
  base_vel.angular.z += key_vel_cmd_.angular.z;

  // Get the next velocity commands, and apply them to the robot.
  double theta = tf::getYaw(base_pose_.pose.orientation);
  double dx = (1.0/getFrameRate())*base_vel.linear.x*std::cos(theta) 
    - (1.0/getFrameRate())*base_vel.linear.y*std::sin(theta);
  double dy = (1.0/getFrameRate())*base_vel.linear.y*std::cos(theta)
    + (1.0/getFrameRate())*base_vel.linear.x*std::sin(theta);
  double dyaw = (1/getFrameRate())*base_vel.angular.z;

  double new_x = base_pose_.pose.position.x + dx;
  double new_y = base_pose_.pose.position.y + dy;
  double new_theta = tf::getYaw(base_pose_.pose.orientation) + dyaw;

  BodyPose body_pose;
  body_pose.x = new_x;
  body_pose.y = new_y;
  body_pose.z = 0.0;
  body_pose.theta = new_theta;

  // Second, get the next proposed end-effector pose and resulting joint angles.
  geometry_msgs::Twist end_effector_vel = 
    end_effector_controller_.moveTo(end_effector_pose_.pose,
				    end_effector_goal_pose_.pose);

  dx = (1.0/getFrameRate())*end_effector_vel.linear.x;
  dy = (1.0/getFrameRate())*end_effector_vel.linear.y;
  double dz = (1.0/getFrameRate())*end_effector_vel.linear.z;

  // Attempt to find joint angles for the arm using the arm IK solver.
  std::vector<double> r_arm_joints(7, 0);
  std::vector<double> l_arm_joints(7, 0);
  for(int i = 7; i < 14; ++i)
  {
    r_arm_joints[i-7] = joint_states_.position[i];
    l_arm_joints[i-7] = joint_states_.position[i-7];
  }

  std::vector<double> end_effector_pose(7, 0);
  end_effector_pose[0] = end_effector_pose_.pose.position.x + dx;
  end_effector_pose[1] = end_effector_pose_.pose.position.y + dy;
  end_effector_pose[2] = end_effector_pose_.pose.position.z + dz;
  end_effector_pose[3] = end_effector_pose_.pose.orientation.x;
  end_effector_pose[4] = end_effector_pose_.pose.orientation.y;
  end_effector_pose[5] = end_effector_pose_.pose.orientation.z;
  end_effector_pose[6] = end_effector_pose_.pose.orientation.w;
  
  // Use IK to find the required joint angles for the arm.
  std::vector<double> solution(7, 0);
  if(!kdl_robot_model_.computeIK(end_effector_pose, r_arm_joints, solution))
  {
    //ROS_ERROR("[PR2Sim] IK failed in move robot!");
  }

  geometry_msgs::Pose object_pose;
  if(attached_object_)
    computeObjectPose(end_effector_pose, body_pose, object_pose);

  if(validityCheck(solution, l_arm_joints, body_pose, object_pose))
  {
    // All is valid, first move the base pose.
    base_pose_.pose.position.x = new_x;
    base_pose_.pose.position.y = new_y;
    base_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(new_theta);

    // Next, set the new joint angles of the right arm and the new 
    // (right) end-effector pose.
    for(int i = 7; i < 14; ++i)
      joint_states_.position[i] = solution[i-7];
  
    end_effector_pose_.pose.position.x = end_effector_pose[0];
    end_effector_pose_.pose.position.y = end_effector_pose[1];
    end_effector_pose_.pose.position.z = end_effector_pose[2];
    end_effector_pose_.pose.orientation.x = end_effector_pose[3];
    end_effector_pose_.pose.orientation.y = end_effector_pose[4];
    end_effector_pose_.pose.orientation.z = end_effector_pose[5];
    end_effector_pose_.pose.orientation.w = end_effector_pose[6];

    if(attached_object_)
      object_manager_->moveObject(attached_id_, object_pose);
  }
  else
  {
    ROS_ERROR("[PR2Sim] Invalid movement!");
  }
}

void PR2Simulator::computeObjectPose(vector<double> eef, BodyPose bp, geometry_msgs::Pose& obj){
  KDL::Frame base_in_map(KDL::Rotation::RotZ(bp.theta),
                         KDL::Vector(bp.x, bp.y, 0.0));
  KDL::Frame eef_in_base(KDL::Rotation::Quaternion(eef[3],eef[4],eef[5],eef[6]),
                         KDL::Vector(eef[0],eef[1],eef[2]));

  KDL::Frame obj_in_map = base_in_map * eef_in_base * attached_transform_;
  obj.position.x = obj_in_map.p.x();
  obj.position.y = obj_in_map.p.y();
  obj.position.z = obj_in_map.p.z();
  obj_in_map.M.GetQuaternion(obj.orientation.x,
                             obj.orientation.y,
                             obj.orientation.z,
                             obj.orientation.w);
}

void PR2Simulator::updateVelocity(const geometry_msgs::Twist &vel)
{
  vel_cmd_ = vel;
}

void PR2Simulator::visualizeRobot()
{
  std::vector<double> l_joints_pos(7, 0);
  std::vector<double> r_joints_pos(8, 0);
  for(int i = 0; i < 7; ++i)
  {
    l_joints_pos[i] = joint_states_.position.at(i);
    r_joints_pos[i] = joint_states_.position.at(i+7);
  }
  r_joints_pos[7] = joint_states_.position.at(14);

  std::vector<double> body_pos(3, 0);
  body_pos[0] = base_pose_.pose.position.x;
  body_pos[1] = base_pose_.pose.position.y;
  body_pos[2] = tf::getYaw(base_pose_.pose.orientation);

  pviz_->visualizeRobot(r_joints_pos, l_joints_pos, body_pos, 0, 0.3, "simple_sim", 0, true);

  BodyPose body;
  body.x = body_pos[0];
  body.y = body_pos[1];
  body.theta = body_pos[2];
  robot_markers_ = pviz_->getRobotMarkerMsg(r_joints_pos, l_joints_pos, body, 0.3, "simple_sim", 0);
}

void PR2Simulator::updateEndEffectorPose()
{
  std::vector<double> r_arm_joints(7, 0);
  for(int i = 0; i < 7; ++i)
    r_arm_joints[i] = joint_states_.position.at(i+7);

  // Publish the pose of the right end effector.
  std::vector<double> fk_pose;
  if(!kdl_robot_model_.computePlanningLinkFK(r_arm_joints, fk_pose))
  {
    ROS_ERROR("Failed to compute FK!");
  }

  end_effector_pose_.pose.position.x = fk_pose[0];
  end_effector_pose_.pose.position.y = fk_pose[1];
  end_effector_pose_.pose.position.z = fk_pose[2];
  end_effector_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(fk_pose[3], 
										fk_pose[4],
										fk_pose[5]);
}

void PR2Simulator::setEndEffectorGoalPose(const geometry_msgs::Pose &goal_pose)
{
  end_effector_goal_pose_.pose = goal_pose;

  if(isValidEndEffectorPose(goal_pose))
    end_effector_controller_.setState(EndEffectorController::READY);
  else
    end_effector_controller_.setState(EndEffectorController::INVALID_GOAL);
}

void PR2Simulator::updateEndEffectorVelocity(const geometry_msgs::Twist &vel)
{
  end_effector_vel_cmd_ = vel;
}

void PR2Simulator::baseMarkerFeedback(
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

void PR2Simulator::gripperMarkerFeedback(
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
      setEndEffectorGoalPose(feedback->pose);
      break;
    }
  default:
    break;
  }
}

void PR2Simulator::setSpeed(double linear, double angular)
{
  if(linear != 0)
  {
    ROS_INFO("[PR2Sim] Setting linear speed to %f.", linear);
    base_movement_controller_.setLinearSpeed(linear);
  }

  if(angular != 0)
  {
    ROS_INFO("[PR2Sim] Setting angular speed to %f.", angular);
    base_movement_controller_.setAngularSpeed(angular);
  }
}

void PR2Simulator::resetRobot()
{
  ROS_INFO("[PR2Sim] Resetting robot...");

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
  int_marker_server_->setPose("base_marker", base_pose_.pose);
  int_marker_server_->applyChanges();

  // Reset the joints. (Leave the left arm joints as they were.)
  for(int i = 7; i < joint_states_.position.size(); ++i)
    joint_states_.position[i] = 0;

  updateEndEffectorPose();

  end_effector_goal_pose_ = end_effector_pose_;

  // Reset the base movement controller.
  base_movement_controller_.setState(BaseMovementController::INITIAL);

  // Reset the end effector controller.
  end_effector_controller_.setState(EndEffectorController::INITIAL);

  // Reset the pose of the end-effector.
  int_marker_server_->setPose("r_gripper_marker", end_effector_pose_.pose);
  int_marker_server_->applyChanges();

  // Delete the drawn base path.
  visualization_msgs::Marker base_path;
  base_path.header.frame_id = "/map";
  base_path.ns = "motion_rec";
  base_path.id = 0;
  base_path.action = visualization_msgs::Marker::DELETE;
  base_path.type = visualization_msgs::Marker::LINE_STRIP;
  marker_pub_.publish(base_path);
}

void PR2Simulator::setRobotPose(const geometry_msgs::Pose &pose)
{
  base_pose_.pose = pose;

  if(base_movement_controller_.getState() == BaseMovementController::INITIAL ||
     base_movement_controller_.getState() == BaseMovementController::DONE)
  {
    int_marker_server_->setPose("base_marker", base_pose_.pose);
    int_marker_server_->applyChanges();
  }
}

void PR2Simulator::setJointStates(const sensor_msgs::JointState &joints)
{
  ROS_ASSERT(joints.name.size() == joints.position.size());

  for(int i = 0; i < joints.name.size(); ++i)
  {
    if(joints_map_.find(joints.name[i]) != joints_map_.end())
    {
      joint_states_.position[joints_map_[joints.name[i]]] = joints.position[i];
    }
    else
      ROS_ERROR("[PR2Sim] Failed to find index for joint \"%s\"!", joints.name[i].c_str());
  }
}

void PR2Simulator::setRobotBaseCommand(const geometry_msgs::Pose &command)
{
  goal_pose_.pose = command;

  int_marker_server_->setPose("base_marker", command);
  int_marker_server_->applyChanges();

  if(base_movement_controller_.getState() == BaseMovementController::INITIAL ||
     base_movement_controller_.getState() == BaseMovementController::DONE)
  {
    base_movement_controller_.setState(BaseMovementController::READY);
  }
}

void PR2Simulator::processKeyEvent(int key, int type)
{
  if(type == QEvent::KeyPress)
  {
    switch(key)
    {
    case Qt::Key_Z:
      {
  	// Change the marker to only move in the +/- z-directions.
  	visualization_msgs::InteractiveMarker gripper_marker;
  	int_marker_server_->get("r_gripper_marker", gripper_marker);

  	if(end_effector_controller_.getState() == EndEffectorController::INITIAL ||
  	   end_effector_controller_.getState() == EndEffectorController::DONE)
  	{
  	  gripper_marker.pose = end_effector_pose_.pose;
  	}
  	else
  	{
  	  gripper_marker.pose = end_effector_goal_pose_.pose;
  	}

  	gripper_marker.controls.at(0).interaction_mode = 
  	  visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  	int_marker_server_->insert(gripper_marker);
  	int_marker_server_->applyChanges();

  	break;
      }
    case Qt::Key_R:
      {
  	// Reset the interactive marker on the gripper to the actual position
  	// of the gripper.
  	updateEndEffectorPose();

  	int_marker_server_->setPose("r_gripper_marker", end_effector_pose_.pose);
  	int_marker_server_->applyChanges();

  	end_effector_goal_pose_ = end_effector_pose_;

  	end_effector_controller_.setState(EndEffectorController::DONE);

  	updateEndEffectorMarker();

  	ROS_INFO("[PR2SimpleSim] Resetting the end-effector marker pose.");

  	break;
      }
    case Qt::Key_W:
      key_vel_cmd_.linear.x = 0.2;
      break;
    case Qt::Key_A:
      key_vel_cmd_.linear.y = 0.2;
      break;
    case Qt::Key_S:
      key_vel_cmd_.linear.x = -0.2;
      break;
    case Qt::Key_D:
      key_vel_cmd_.linear.y = -0.2;
      break;
    default:
      break;
    }
  }
  else if(type == QEvent::KeyRelease)
  {
    switch(key)
    {
    case Qt::Key_Z:
      {
  	// Change the marker back to moving in the xy-plane.
  	visualization_msgs::InteractiveMarker gripper_marker;
  	int_marker_server_->get("r_gripper_marker", gripper_marker);

  	if(end_effector_controller_.getState() == EndEffectorController::INITIAL ||
  	   end_effector_controller_.getState() == EndEffectorController::DONE)
  	{
  	  gripper_marker.pose = end_effector_pose_.pose;
  	}
  	else
  	{
  	  gripper_marker.pose = end_effector_goal_pose_.pose;
  	}

  	gripper_marker.controls.at(0).interaction_mode = 
  	  visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  	int_marker_server_->insert(gripper_marker);
  	int_marker_server_->applyChanges();

  	break;
      }
    case Qt::Key_W:
      key_vel_cmd_.linear.x = 0;
      break;
    case Qt::Key_A:
      key_vel_cmd_.linear.y = 0;
      break;
    case Qt::Key_S:
      key_vel_cmd_.linear.x = 0;
      break;
    case Qt::Key_D:
      key_vel_cmd_.linear.y = 0;
      break;
    default:
      break;
    }
  }
  else
    ROS_ERROR("[PR2Sim] Invalid key event type!");
}

void PR2Simulator::updateEndEffectorMarker()
{
  // If the gripper marker is at an invalid pose, it should turn red.
  if(end_effector_controller_.getState() == EndEffectorController::INVALID_GOAL &&
     end_effector_controller_.getLastState() != EndEffectorController::INVALID_GOAL)
  {
    visualization_msgs::InteractiveMarker gripper_marker;
    int_marker_server_->get("r_gripper_marker", gripper_marker);
    gripper_marker.controls[0].markers[0].color.r = 1;
    gripper_marker.controls[0].markers[0].color.g = 0;
    gripper_marker.controls[0].markers[0].color.b = 0;
    gripper_marker.pose = end_effector_goal_pose_.pose;
    int_marker_server_->insert(gripper_marker);
    int_marker_server_->applyChanges();
  }
  else if(end_effector_controller_.getState() != EndEffectorController::INVALID_GOAL &&
	  end_effector_controller_.getLastState() == EndEffectorController::INVALID_GOAL)
  {
    visualization_msgs::InteractiveMarker gripper_marker;
    int_marker_server_->get("r_gripper_marker", gripper_marker);
    gripper_marker.controls[0].markers[0].color.r = 0;
    gripper_marker.controls[0].markers[0].color.g = 1;
    gripper_marker.controls[0].markers[0].color.b = 0;
    gripper_marker.pose = end_effector_goal_pose_.pose;
    int_marker_server_->insert(gripper_marker);
    int_marker_server_->applyChanges();
  }

  if(!is_moving_r_gripper_ 
     && end_effector_controller_.getState() == EndEffectorController::DONE
     || recorder_->isReplaying())
  {
    int_marker_server_->setPose("r_gripper_marker", end_effector_pose_.pose);
    int_marker_server_->applyChanges();
    end_effector_controller_.setState(EndEffectorController::INITIAL);
  }

  if(end_effector_marker_vel_.linear.x == 0 &&
     end_effector_marker_vel_.linear.y == 0 &&
     end_effector_marker_vel_.linear.z == 0)
    return;

  // Get the current pose of the end-effector interactive marker.
  visualization_msgs::InteractiveMarker marker;
  int_marker_server_->get("r_gripper_marker", marker);
  geometry_msgs::Pose pose = marker.pose;

  pose.position.x += (1/getFrameRate())*end_effector_marker_vel_.linear.x;
  pose.position.y += (1/getFrameRate())*end_effector_marker_vel_.linear.y;
  pose.position.z += (1/getFrameRate())*end_effector_marker_vel_.linear.z;

  // Update the pose according to the current velocity command.
  int_marker_server_->setPose("r_gripper_marker", pose);
  int_marker_server_->applyChanges();

  // Update the goal pose.
  setEndEffectorGoalPose(pose);

  end_effector_controller_.setState(EndEffectorController::READY);
}


void PR2Simulator::updateEndEffectorMarkerVelocity(const geometry_msgs::Twist &vel)
{
  end_effector_marker_vel_ = vel;
}

void PR2Simulator::updateTransforms()
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

  // Get the pose of the base footprint in the torso lift link frame.
  KDL::Frame base_in_torso_lift_link;
  base_in_torso_lift_link.p.x(0.050);
  base_in_torso_lift_link.p.y(0.0);
  base_in_torso_lift_link.p.z(-0.802);
  base_in_torso_lift_link.M = KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

  // Note that all computed poses of the end-effector are in the base footprint frame.
  kdl_robot_model_.setKinematicsToPlanningTransform(base_in_torso_lift_link.Inverse(), "base_footprint");
}

bool PR2Simulator::isBaseMoving() const
{
  if((base_movement_controller_.getState() == BaseMovementController::DONE ||
     base_movement_controller_.getState() == BaseMovementController::INITIAL) &&
     vel_cmd_.linear.x == 0 && vel_cmd_.linear.y == 0 && vel_cmd_.angular.z == 0 &&
     key_vel_cmd_.linear.x == 0 && key_vel_cmd_.linear.y == 0 && key_vel_cmd_.angular.z == 0)
    return false;

  return true;
}

bool PR2Simulator::isValidEndEffectorPose(const geometry_msgs::Pose &pose)
{
  // Attempt to compute IK for the arm given a goal pose and the current 
  // configuration.
  std::vector<double> r_arm_joints(7, 0);
  for(int i = 7; i < 14; ++i)
  {
    r_arm_joints[i-7] = joint_states_.position[i];
  }

  std::vector<double> goal_end_effector_pose(7, 0);
  goal_end_effector_pose[0] = pose.position.x;
  goal_end_effector_pose[1] = pose.position.y;
  goal_end_effector_pose[2] = pose.position.z;
  goal_end_effector_pose[3] = pose.orientation.x;
  goal_end_effector_pose[4] = pose.orientation.y;
  goal_end_effector_pose[5] = pose.orientation.z;
  goal_end_effector_pose[6] = pose.orientation.w;
  
  // Use IK to find the required joint angles for the arm. If it fails, then this
  // is not a valid end-effector pose.
  std::vector<double> solution(7, 0);
  if(!kdl_robot_model_.computeIK(goal_end_effector_pose, r_arm_joints, solution))
  {
    return false;
  }
  return true;
}

void PR2Simulator::attach(int id, KDL::Frame transform){
  attached_object_ = true;
  attached_id_ = id;
  attached_transform_ = transform;
}

void PR2Simulator::detach(){
  attached_object_ = false;
}

bool PR2Simulator::validityCheck(vector<double> rangles, vector<double> langles, BodyPose bp, geometry_msgs::Pose object_pose){
  if(attached_object_){
    //check robot motion
    if(!object_manager_->checkRobotMove(rangles, langles, bp, attached_id_))
      return false;
    
    if(!object_manager_->checkObjectMove(attached_id_, object_pose, rangles, langles, bp))
      return false;
  }
  else{
    //check robot motion
    if(!object_manager_->checkRobotMove(rangles, langles, bp, -1))
      return false;
  }
  return true;
}

void PR2Simulator::showEndEffectorWorkspaceArc()
{
  visualization_msgs::Marker arc;

  arc.header.stamp = ros::Time::now();
  arc.header.frame_id = "/map";

  // Align the center of the arc with right shoulder link.
  arc.pose.position.x = robot_markers_.markers.at(2).pose.position.x;
  arc.pose.position.y = robot_markers_.markers.at(2).pose.position.y;
  arc.pose.position.z = 0;
  arc.pose.orientation = robot_markers_.markers.at(2).pose.orientation;

  arc.ns = "pr2_simple_sim";
  arc.id = 0;
  arc.type = visualization_msgs::Marker::LINE_STRIP;
  arc.action = visualization_msgs::Marker::ADD;
  arc.scale.x = 0.03;
  arc.color.r = 1;
  arc.color.g = 0;
  arc.color.b = 0;
  arc.color.a = 0.4;
  
  double y = 0;
  double x = 0;
  for(int i = 0; i < 200; ++i)
  {
    y = (0.74 / 100.0) * i - 0.74;
    x = std::sqrt(std::pow(0.74, 2) - std::pow(y, 2));
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    arc.points.push_back(p);
  }

  marker_pub_.publish(arc);
}

geometry_msgs::Pose PR2Simulator::getBasePose() const
{
  return base_pose_.pose;
}

geometry_msgs::Pose PR2Simulator::getEndEffectorPose()
{
  // Transform the end effector pose into the map frame and return it.
  tf::Transform base_footprint_in_map(tf::Quaternion(base_pose_.pose.orientation.x,
						     base_pose_.pose.orientation.y,
						     base_pose_.pose.orientation.z,
						     base_pose_.pose.orientation.w),
				      tf::Vector3(base_pose_.pose.position.x,
						  base_pose_.pose.position.y,
						  base_pose_.pose.position.z)
				      );
  
  tf::Transform end_effector_in_base(tf::Quaternion(end_effector_pose_.pose.orientation.x,
						    end_effector_pose_.pose.orientation.y,
						    end_effector_pose_.pose.orientation.z,
						    end_effector_pose_.pose.orientation.w),
				     tf::Vector3(end_effector_pose_.pose.position.x,
						 end_effector_pose_.pose.position.y,
						 end_effector_pose_.pose.position.z)
				     );

  tf::Transform end_effector_in_map = base_footprint_in_map * end_effector_in_base;

  geometry_msgs::Pose end_effector_pose;
  tf::quaternionTFToMsg(end_effector_in_map.getRotation(), end_effector_pose.orientation);
  geometry_msgs::Vector3 position;
  tf::vector3TFToMsg(end_effector_in_map.getOrigin(), position);
  end_effector_pose.position.x = position.x;
  end_effector_pose.position.y = position.y;
  end_effector_pose.position.z = position.z;

  return end_effector_pose;
}

geometry_msgs::Pose PR2Simulator::getEndEffectorMarkerPose()
{
  visualization_msgs::InteractiveMarker marker;
  int_marker_server_->get("r_gripper_marker", marker);
  geometry_msgs::Pose marker_pose = marker.pose;

  tf::Transform base_footprint_in_map(tf::Quaternion(base_pose_.pose.orientation.x,
						     base_pose_.pose.orientation.y,
						     base_pose_.pose.orientation.z,
						     base_pose_.pose.orientation.w),
				      tf::Vector3(base_pose_.pose.position.x,
						  base_pose_.pose.position.y,
						  base_pose_.pose.position.z)
				      );
  
  tf::Transform marker_in_base(tf::Quaternion(marker_pose.orientation.x,
					      marker_pose.orientation.y,
					      marker_pose.orientation.z,
					      marker_pose.orientation.w),
			       tf::Vector3(marker_pose.position.x,
					   marker_pose.position.y,
					   marker_pose.position.z)
			       );

  tf::Transform marker_in_map = base_footprint_in_map * marker_in_base;

  geometry_msgs::Pose end_effector_marker_pose;
  tf::quaternionTFToMsg(marker_in_map.getRotation(), end_effector_marker_pose.orientation);
  geometry_msgs::Vector3 position;
  tf::vector3TFToMsg(marker_in_map.getOrigin(), position);
  end_effector_marker_pose.position.x = position.x;
  end_effector_marker_pose.position.y = position.y;
  end_effector_marker_pose.position.z = position.z;  

  return end_effector_marker_pose;
}

} // namespace demonstration_visualizer
