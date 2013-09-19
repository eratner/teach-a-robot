#include <dviz_core/pr2_simulator.h>

namespace demonstration_visualizer {

PR2Simulator::PR2Simulator(MotionRecorder *recorder, 
			   PViz *pviz,
			   interactive_markers::InteractiveMarkerServer *int_marker_server,
			   ObjectManager* object_manager,
                           int user_id)
  : playing_(true), 
    user_id_(user_id),
    move_end_effector_while_dragging_(true),
    move_base_while_dragging_(false),
    attached_object_(false),
    frame_rate_(10.0), 
    pviz_(pviz), 
    int_marker_server_(int_marker_server), 
    object_manager_(object_manager),
    base_movement_controller_(),
    end_effector_controller_(int_marker_server),
    recorder_(recorder),
    snap_motion_count_(0),
    snap_object_(false),
    stop_while_snapping_(true),
    moving_gripper_marker_(false),
    move_robot_markers_(true),
    pause_requested_(false),
    goal_orientation_changed_(false),
    delta_arm_roll_(0),
    ignore_collisions_(false)
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
  base_pose_.header.frame_id = resolveName("map", user_id_);
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
  joint_states_.position[1] = 1.178526;
  joint_states_.position[2] = 1.789070;
  joint_states_.position[3] = -1.683386;
  joint_states_.position[4] = -1.7343417;
  joint_states_.position[5] = -0.0962141;
  joint_states_.position[6] = -0.0864407;
  // Move the right arm in slightly. @todo these should be constants somewhere.
  joint_states_.position[7] = -0.002109;
  joint_states_.position[8] = 0.655300;
  joint_states_.position[9] = 0.000000;
  joint_states_.position[10] = -1.517650;
  joint_states_.position[11] = -3.138816;
  joint_states_.position[12] = -0.862352;
  joint_states_.position[13] = 3.139786;
 
  // Keep left arm straight down (Only valid if torso is raised all the way)
  joint_states_.position[0] = 0.2;
  joint_states_.position[1] = 1.4;
  joint_states_.position[2] = 1.9;
  joint_states_.position[3] = -0.4;
  joint_states_.position[4] = -0.1;
  joint_states_.position[5] = -1.00;
  joint_states_.position[6] = 0.0;


  // Set the gripper to be open initially.
  joint_states_.position[14] = EndEffectorController::GRIPPER_OPEN_ANGLE;

  for(int i = 0; i < 15; ++i)
    joint_states_.velocity[i] = joint_states_.effort[i] = 0;

  // Create a mapping from joint names to index.
  for(int i = 0; i < 15; ++i)
    joints_map_[joint_states_.name[i]] = i;

  // Set the initial position of the torso to be fully raised. @todo make this a constant
  torso_position_ = 0.3;

  vel_cmd_sub_ = nh.subscribe(resolveName("vel_cmd", user_id_),
			      100,
			      &PR2Simulator::updateBaseVelocity,
			      this);

  end_effector_vel_cmd_sub_ = nh.subscribe(resolveName("end_effector_vel_cmd", user_id_),
					   100,
					   &PR2Simulator::updateEndEffectorVelocity,
					   this);

  marker_pub_ = nh.advertise<visualization_msgs::Marker>(
    resolveName("visualization_marker", user_id_), 1000);

  // Attach an interactive marker to the base of the robot.
  visualizeRobot();
  
  // Initialize the end-effector pose.
  end_effector_pose_.header.frame_id = resolveName("base_footprint", user_id_);
  end_effector_pose_.header.stamp = ros::Time();
  end_effector_pose_.pose = robot_markers_.markers.at(11).pose;

  end_effector_goal_pose_ = end_effector_pose_;

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = resolveName("map", user_id_);
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
  
  control.markers[0].pose = geometry_msgs::Pose();
  control.markers[0].pose.orientation.w = 1;
  control.markers[0].header = std_msgs::Header();

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

  int_marker.header.frame_id = resolveName("base_footprint", user_id_);
  int_marker.pose = robot_markers_.markers.at(11).pose;
  int_marker.name = "r_gripper_marker";
  int_marker.description = "";
  int_marker.scale = 0.3;

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
  control.markers[0].pose.orientation.w = 1;
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

  ROS_DEBUG("[PR2Sim] About to initialize the kinematic model.");
  std::string robot_description;
  std::string robot_param = "";
  if(!nh.searchParam("robot_description", robot_param))
  {
    ROS_ERROR("[PR2Sim] Failed to find the robot_description on the parameter server.");
  }
  nh.param<std::string>(robot_param, robot_description, "");
  std::vector<std::string> planning_joints(joint_states_.name.begin()+7, joint_states_.name.end()-1);
  if(!kdl_robot_model_.init(robot_description, planning_joints))
    ROS_ERROR("[PR2Sim] Failed to initialize the KDLRobotModel for the PR2."); 

  kdl_robot_model_.setPlanningLink("r_gripper_palm_link");

  // Set the map to torso_lift_link transform.
  updateTransforms();
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
  pause_requested_ = false;
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

  // @todo fix this, force key releases.
  processKeyEvent(Qt::Key_W, QEvent::KeyRelease);
  processKeyEvent(Qt::Key_A, QEvent::KeyRelease);
  processKeyEvent(Qt::Key_S, QEvent::KeyRelease);
  processKeyEvent(Qt::Key_D, QEvent::KeyRelease);
  processKeyEvent(Qt::Key_Z, QEvent::KeyRelease);
}

void PR2Simulator::pauseLater()
{
  pause_requested_ = true;
}

bool PR2Simulator::isPlaying() const
{
  return playing_;
}

void PR2Simulator::run()
{
  // If moving the robot using interactive markers (i.e. the base and end-effector
  // carrots) is disabled, then ensure that the carrots stay in place.
  if(!canMoveRobotMarkers())
  {
    int_marker_server_->setPose("base_marker", base_pose_.pose, base_pose_.header);
    int_marker_server_->setPose("r_gripper_marker", end_effector_pose_.pose, end_effector_pose_.header);
    int_marker_server_->applyChanges();     
  }

  if(isPlaying())
  {
    // ROS_INFO_STREAM("Base moving? " << (isBaseMoving() ? "Yes." : "No."));
    // ROS_INFO_STREAM("End-effector moving? " << (isEndEffectorMoving() ? "Yes." : "No."));

    // Perform any snap motion that has been generated.
    if(!isSnapDone())
    {
      // ROS_INFO("[PR2Sim] Executing a snap motion (%d)...", snap_motion_count_);
      snap_motion_count_++;
    }
    else if(pause_requested_)
    {
      pause();
      pause_requested_ = false;
      return;
    }

    moveRobot();

    visualizeRobot();

    updateTransforms();

    updateEndEffectorPose();

    updateEndEffectorMarker();

    updateUpperArmMarker();

    // Record motion.
    if(recorder_->isRecording())
    {
      recorder_->recordBasePose(base_pose_);
      recorder_->recordJoints(joint_states_);
    }

    // Replay motion.
    if(recorder_->isReplaying())
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

	int_marker_server_->setPose("base_marker", base_pose_.pose, base_pose_.header);
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

  if(base_movement_controller_.getState() == BaseMovementController::DONE ||
     (isBaseMoving() && base_movement_controller_.getState() == BaseMovementController::INITIAL)
     )
  {
    int_marker_server_->setPose("base_marker", base_pose_.pose, base_pose_.header);
    int_marker_server_->applyChanges();
  }

  if(base_movement_controller_.getState() == BaseMovementController::DONE)
  {
    base_movement_controller_.setState(BaseMovementController::INITIAL);
  }
  
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
  body_pose.z = getTorsoPosition();
  body_pose.theta = new_theta;

  double dz = 0;
  std::vector<double> end_effector_pose(7, 0);
  std::vector<double> solution(7, 0);
  std::vector<double> l_arm_joints(7, 0);
  for(int i = 0; i < 7; ++i)
  {
    l_arm_joints[i] = joint_states_.position[i];
  }

  if(isSnapDone())
  {
    // Second, get the next proposed end-effector pose and resulting joint angles.
    geometry_msgs::Twist end_effector_vel = 
      end_effector_controller_.moveTo(end_effector_pose_.pose,
				      end_effector_goal_pose_.pose);
    
    dx = (1.0/getFrameRate())*end_effector_vel.linear.x + (1.0/getFrameRate())*end_effector_vel_cmd_.linear.x;
    dy = (1.0/getFrameRate())*end_effector_vel.linear.y + (1.0/getFrameRate())*end_effector_vel_cmd_.linear.y;
    dz = (1.0/getFrameRate())*end_effector_vel.linear.z + (1.0/getFrameRate())*end_effector_vel_cmd_.linear.z;

    // Attempt to find joint angles for the arm using the arm IK solver.
    std::vector<double> r_arm_joints(7, 0);
    for(int i = 7; i < 14; ++i)
    {
      r_arm_joints[i-7] = joint_states_.position[i];
    }

    // std::vector<double> end_effector_pose(7, 0);
    end_effector_pose[0] = end_effector_pose_.pose.position.x + dx;
    end_effector_pose[1] = end_effector_pose_.pose.position.y + dy;
    end_effector_pose[2] = end_effector_pose_.pose.position.z + dz;
    end_effector_pose[3] = end_effector_pose_.pose.orientation.x;
    end_effector_pose[4] = end_effector_pose_.pose.orientation.y;
    end_effector_pose[5] = end_effector_pose_.pose.orientation.z;
    end_effector_pose[6] = end_effector_pose_.pose.orientation.w;
  
    // Use IK to find the required joint angles for the arm.
    if(!kdl_robot_model_.computeIK(end_effector_pose, r_arm_joints, solution))
    {
      // Do not attempt an IK search if the simulator is executing a snap motion.
      if(!isSnapDone())
      {
	ROS_ERROR("[PR2Sim] IK failed in move robot while snapping!");
	return;
      }

      // Try once more to find a valid IK solution by searching locally for valid
      // end-effector positions.
      ROS_INFO("[PR2Sim] IK failed at (%f, %f, %f), but...", end_effector_pose[0], 
	       end_effector_pose[1], end_effector_pose[2]);
      std::vector<std::pair<double, double> > intervals;
      intervals.push_back(std::make_pair(-0.03 + end_effector_pose[0], 
					 0.03 + end_effector_pose[0]));
      intervals.push_back(std::make_pair(-0.03 + end_effector_pose[1], 
					 0.03 + end_effector_pose[1]));
      intervals.push_back(std::make_pair(end_effector_pose[2], 
					 end_effector_pose[2]));
      std::vector<double> d;
      d.push_back(0.002);
      d.push_back(0.002);
      d.push_back(0);
      geometry_msgs::Pose pose;
      pose.position.x = end_effector_pose[0];
      pose.position.y = end_effector_pose[1];
      pose.position.z = end_effector_pose[2];
      pose.orientation = end_effector_pose_.pose.orientation;
      double x, y, z;

      if(closestValidEndEffectorPosition(pose, intervals, d, x, y, z))
      {
	ROS_INFO("...found a valid position at (%f, %f, %f)!", x, y, z);
	end_effector_pose[0] = x;
	end_effector_pose[1] = y;
	end_effector_pose[2] = z;
	if(!kdl_robot_model_.computeIK(end_effector_pose, r_arm_joints, solution))
	{
	  ROS_ERROR("This should never happen!");
	  return;
	}
      }
      else
      {
	ROS_INFO("...nevermind :(");
	return;
      }
    }
  }
  else
  {
    for(int i = 0; i < 7; ++i)
    {
      solution[i] = snap_motion_[snap_motion_count_].position[i];
    }
    joint_states_.position[14] = snap_motion_[snap_motion_count_].position[7];

    // Compute FK to find the end-effector pose.
    std::vector<double> fk_pose;
    if(!kdl_robot_model_.computePlanningLinkFK(solution, fk_pose))
    {
      ROS_ERROR("[PR2Sim] Failed to compute FK in while executing a snap motion!");
    }
    else
    {
      end_effector_pose[0] = fk_pose[0];
      end_effector_pose[1] = fk_pose[1];
      end_effector_pose[2] = fk_pose[2];
      geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromRollPitchYaw(fk_pose[3], 
										      fk_pose[4],
										      fk_pose[5]);
      end_effector_pose[3] = orientation.x;
      end_effector_pose[4] = orientation.y;
      end_effector_pose[5] = orientation.z;
      end_effector_pose[6] = orientation.w;
    }
  }
  
  bool upper_arm_roll_changed = false;
  // upper arm roll
  if(delta_arm_roll_ != 0)
  {
    upper_arm_roll_changed = true;

    double step_size = 0.02;
    if(std::abs(delta_arm_roll_) < step_size)
    {
      step_size = std::abs(delta_arm_roll_);
    }

    if(delta_arm_roll_ < 0)
      step_size *= -1.0;

    // ROS_INFO("step_size = %f, delta_arm_roll = %f", step_size, delta_arm_roll_);

    // solution[2] += step_size;
    delta_arm_roll_ -= step_size;
    
    std::vector<double> r_arm_joints = solution;
    r_arm_joints[2] += step_size;
    std::vector<double> roll_solution(7, 0);
    
    if(!kdl_robot_model_.computeIK(end_effector_pose, r_arm_joints, roll_solution))
    {
      ROS_ERROR("[PR2Sim] IK failed on changing the right upper arm roll!");
      delta_arm_roll_ = 0;
    }
    else
    {
      solution = roll_solution;
    }
  }

  geometry_msgs::Pose object_pose;
  if((attached_object_ && (isSnapDone() || snap_object_)) || 
     (!isSnapDone() && snap_object_))
  {
    computeObjectPose(end_effector_pose, body_pose, object_pose);
  }
  else if(attached_object_)
  {
    object_pose = object_manager_->getMarker(attached_id_).pose;
  }

  if(validityCheck(solution, l_arm_joints, body_pose, object_pose))
  {
    ROS_DEBUG("[PR2Sim] Collision free!");
    // All is valid, first move the base pose.
    base_pose_.pose.position.x = new_x;
    base_pose_.pose.position.y = new_y;
    base_pose_.pose.orientation = tf::createQuaternionMsgFromYaw(new_theta);

    // Next, set the new joint angles of the right arm and the new 
    // (right) end-effector pose.
    for(int i = 7; i < 14; ++i)
    {
      joint_states_.position[i] = solution[i-7];
    }

    end_effector_pose_.pose.position.x = end_effector_pose[0];
    end_effector_pose_.pose.position.y = end_effector_pose[1];
    end_effector_pose_.pose.position.z = end_effector_pose[2];
    end_effector_pose_.pose.orientation.x = end_effector_pose[3];
    end_effector_pose_.pose.orientation.y = end_effector_pose[4];
    end_effector_pose_.pose.orientation.z = end_effector_pose[5];
    end_effector_pose_.pose.orientation.w = end_effector_pose[6];

    if((attached_object_ && (isSnapDone() || snap_object_)) || 
       (!isSnapDone() && snap_object_))
    {
      object_manager_->moveObject(attached_id_, object_pose);
    }

    if(upper_arm_roll_changed)
    {
      visualization_msgs::InteractiveMarker marker;
      int_marker_server_->get("r_gripper_marker", marker);
      geometry_msgs::Pose pose = marker.pose;

      tf::Quaternion rot(pose.orientation.x, 
			 pose.orientation.y,
			 pose.orientation.z,
			 pose.orientation.w);
      tf::Quaternion rot2(tf::Vector3(0, 1.0, 0), M_PI/2.0);
      tf::quaternionTFToMsg(rot.inverse() * rot2, marker.controls.at(0).orientation);
      
      int_marker_server_->insert(marker);
      int_marker_server_->applyChanges();
    }
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

  // double r, p, y;
  // eef_in_base.M.GetRPY(r, p, y);
  // ROS_INFO("computeObjectPose() end effector pose in base (%f, %f, %f), (%f, %f, %f)",
  // 	   eef[0], eef[1], eef[2], r, p, y);

  KDL::Frame obj_in_map = base_in_map * eef_in_base * attached_transform_;
  obj.position.x = obj_in_map.p.x();
  obj.position.y = obj_in_map.p.y();
  obj.position.z = obj_in_map.p.z();
  obj_in_map.M.GetQuaternion(obj.orientation.x,
                             obj.orientation.y,
                             obj.orientation.z,
                             obj.orientation.w);
}

void PR2Simulator::updateBaseVelocity(const geometry_msgs::Twist &vel)
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
    // ROS_INFO("right arm joint %d: %f", i+7, r_joints_pos[i]);
  }
  r_joints_pos[7] = joint_states_.position.at(14);

  std::vector<double> body_pos(3, 0);
  body_pos[0] = base_pose_.pose.position.x;
  body_pos[1] = base_pose_.pose.position.y;
  body_pos[2] = tf::getYaw(base_pose_.pose.orientation);

  pviz_->visualizeRobot(r_joints_pos, l_joints_pos, body_pos, getTorsoPosition(), 0.3, "simple_sim", 0, true);

  BodyPose body;
  body.x = body_pos[0];
  body.y = body_pos[1];
  body.z = getTorsoPosition();
  body.theta = body_pos[2];
  robot_markers_ = pviz_->getRobotMarkerMsg(r_joints_pos, l_joints_pos, body, 0.3, "simple_sim", 0);
}

void PR2Simulator::updateEndEffectorPose()
{
  std::vector<double> r_arm_joints(7, 0);
  for(int i = 0; i < 7; ++i)
  {
    r_arm_joints[i] = joint_states_.position.at(i+7);
    // ROS_INFO("Joint %s has angle %f", joint_states_.name.at(i+7).c_str(), joint_states_.position.at(i+7));
  }

  // Publish the pose of the right end effector.
  std::vector<double> fk_pose;
  if(!kdl_robot_model_.computePlanningLinkFK(r_arm_joints, fk_pose))
  {
    ROS_ERROR("[PR2Sim] Failed to compute FK in updating the end-effector pose!");
  }

  // ROS_INFO("Updating end-effector pose to (%f, %f, %f), (%f, %f, %f).", fk_pose[0], fk_pose[1],
  // 	   fk_pose[2], fk_pose[3], fk_pose[4], fk_pose[5]);

  end_effector_pose_.pose.position.x = fk_pose[0];
  end_effector_pose_.pose.position.y = fk_pose[1];
  end_effector_pose_.pose.position.z = fk_pose[2];
  end_effector_pose_.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(fk_pose[3], 
										fk_pose[4],
										fk_pose[5]);
}

bool PR2Simulator::setEndEffectorGoalPose(const geometry_msgs::Pose &goal_pose)
{
  end_effector_goal_pose_.pose = goal_pose;

  if(isValidEndEffectorPose(goal_pose))
  {
    end_effector_controller_.setState(EndEffectorController::READY);
    return true;
  }

  end_effector_controller_.setState(EndEffectorController::INVALID_GOAL);
  return false;
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
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
  {
    if(move_base_while_dragging_ && canMoveRobotMarkers())
    {
      base_movement_controller_.setState(BaseMovementController::READY);
      goal_pose_.pose = feedback->pose;
    }
    break;
  }
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    break;
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
  {
    if(!move_base_while_dragging_ && canMoveRobotMarkers())
    {
      base_movement_controller_.setState(BaseMovementController::READY);
      goal_pose_.pose = feedback->pose;
      // double yaw = std::atan2(goal_pose_.pose.position.y - base_pose_.pose.position.y,
      // 			      goal_pose_.pose.position.x - base_pose_.pose.position.x);
      // ROS_INFO("[PR2SimpleSim] New goal set at (x, y, yaw) = (%f, %f, %f).", 
      // 	       goal_pose_.pose.position.x,
      // 	       goal_pose_.pose.position.y,
      // 	       yaw * (180.0/M_PI));
    }
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
  case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
  {
    if(move_end_effector_while_dragging_ && canMoveRobotMarkers())
    {
      // ROS_INFO("pose update at (%f, %f, %f)!", feedback->pose.position.x, 
      // 	     feedback->pose.position.y, feedback->pose.position.z);
      setEndEffectorGoalPose(feedback->pose);
      end_effector_goal_pose_.pose = feedback->pose;
    }
    break;
  }
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    break;
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
  {
    // ROS_INFO("control name: %s", feedback->control_name.c_str());
    if(feedback->control_name.compare("ROLL") == 0 ||
       feedback->control_name.compare("PITCH") == 0 ||
       feedback->control_name.compare("YAW") == 0)
    {
      // snapEndEffectorTo(feedback->pose, EndEffectorController::GRIPPER_OPEN_ANGLE,
      // 		  false, false, true);
      goal_orientation_changed_ = true;
    }

    if(!move_end_effector_while_dragging_ && canMoveRobotMarkers())
    {
      ROS_INFO("[PR2Sim] Setting new end effector goal position at (%f, %f, %f).",
	       feedback->pose.position.x, feedback->pose.position.y, feedback->pose.position.z);
      setEndEffectorGoalPose(feedback->pose);
    }
    break;
  }
  default:
    break;
  }
}

void PR2Simulator::upperArmMarkerFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
)
{
  switch(feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
  {
    double current_roll = 0;
    double next_roll = 0;

    double r, p, y;
    KDL::Rotation next_rot = KDL::Rotation::Quaternion(feedback->pose.orientation.x,
						       feedback->pose.orientation.y,
						       feedback->pose.orientation.z,
						       feedback->pose.orientation.w);
    next_rot.GetRPY(r, p, y);
    next_roll = r;
    ROS_INFO("Next RPY: (%f, %f, %f)", r, p, y);
    KDL::Rotation current_rot = KDL::Rotation::Quaternion(r_upper_arm_roll_pose_.orientation.x,
							  r_upper_arm_roll_pose_.orientation.y,
							  r_upper_arm_roll_pose_.orientation.z,
							  r_upper_arm_roll_pose_.orientation.w);
    current_rot.GetRPY(r, p, y);
    current_roll = r;
    // ROS_INFO("Current RPY: (%f, %f, %f)", r, p, y);

    // ROS_INFO("Current r_upper_arm_roll_joint value = %f", joint_states_.position[9]);

    double angle = next_roll - current_roll;

    delta_arm_roll_ = -1.0 * angles::normalize_angle(angle);

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

  geometry_msgs::Pose origin;
  origin.position.x = 0;
  origin.position.y = 0;
  origin.position.z = 0;
  origin.orientation.x = 0;
  origin.orientation.y = 0;
  origin.orientation.z = 0;
  origin.orientation.w = 1;

  resetRobotTo(origin);
}

void PR2Simulator::resetRobotTo(const geometry_msgs::Pose &pose, double torso_position)
{
  ROS_INFO("[PR2Sim] Resetting robot to (%f, %f, %f), (%f, %f, %f, %f)...",
	   pose.position.x, pose.position.y, pose.position.y,
	   pose.orientation.x, pose.orientation.y, pose.orientation.z, 
	   pose.orientation.w);

  torso_position_ = torso_position;

  // Reset the base pose.
  base_pose_.pose = pose;

  // Reset the goal pose.
  goal_pose_ = base_pose_;
  
  // Reset the base pose interactive marker.
  int_marker_server_->setPose("base_marker", base_pose_.pose, base_pose_.header);
  int_marker_server_->applyChanges();

  // Reset the joints. (Leave the left arm joints as they were.)
  joint_states_.position[7] = -0.002109;
  joint_states_.position[8] = 0.655300;
  joint_states_.position[9] = 0.000000;
  joint_states_.position[10] = -1.517650;
  joint_states_.position[11] = -3.138816;
  joint_states_.position[12] = -0.862352;
  // joint_states_.position[13] = 3.139786;
  joint_states_.position[13] = 0.0;

  joint_states_.position[14] = EndEffectorController::GRIPPER_OPEN_ANGLE;

  updateEndEffectorPose();

  end_effector_goal_pose_ = end_effector_pose_;

  // Reset the base movement controller.
  base_movement_controller_.setState(BaseMovementController::INITIAL);

  // Reset the end effector controller.
  end_effector_controller_.setState(EndEffectorController::INITIAL);

  // Reset the pose of the end-effector.
  int_marker_server_->setPose("r_gripper_marker", end_effector_pose_.pose, end_effector_pose_.header);
  int_marker_server_->applyChanges();

  if(attached_object_)
    detach();

  // Delete the drawn base path.
  visualization_msgs::Marker base_path;
  base_path.header.frame_id = resolveName("map", user_id_);
  base_path.ns = "motion_rec";
  base_path.id = 0;
  base_path.action = visualization_msgs::Marker::DELETE;
  base_path.type = visualization_msgs::Marker::LINE_STRIP;
  marker_pub_.publish(base_path);

  // If the simulator is paused, we need to manually update the 
  // visualization of the robot.
  if(!isPlaying())
  {
    visualizeRobot();
  }  
}

void PR2Simulator::setRobotPose(const geometry_msgs::Pose &pose)
{
  base_pose_.pose = pose;

  if(base_movement_controller_.getState() == BaseMovementController::INITIAL ||
     base_movement_controller_.getState() == BaseMovementController::DONE)
  {
    int_marker_server_->setPose("base_marker", base_pose_.pose, base_pose_.header);
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
      // ROS_INFO("[PR2Sim] Setting joint \"%s\" to angle %f.", joints.name[i].c_str(), joints.position[i]);
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

bool PR2Simulator::snapEndEffectorTo(const geometry_msgs::Pose &pose, 
				     double gripper_joint, 
				     bool snap_attached_object,
                                     bool interpolate_position,
                                     bool interpolate_orientation,
                                     bool stop_while_snapping)
{
  if(isValidEndEffectorPose(pose))
  {
    snap_object_ = snap_attached_object;
    stop_while_snapping_ = stop_while_snapping;

    // @todo sometimes the key release command gets lost
    end_effector_vel_cmd_.linear.x = 0;
    end_effector_vel_cmd_.linear.y = 0;
    end_effector_vel_cmd_.linear.z = 0;

    geometry_msgs::Pose current_pose = end_effector_pose_.pose;

    // // DEBUGGING: Find the shortest angular distance between the current and goal 
    // // orientation quaternions.
    // tf::Quaternion curr;
    // tf::quaternionMsgToTF(current_pose.orientation, curr);
    // tf::Quaternion goal;
    // tf::quaternionMsgToTF(pose.orientation, goal);
    // double shortest = (double)curr.angleShortestPath(goal);
    // ROS_ERROR("shortest angular path = %f", shortest);

    // Generate an interpolation of end-effector poses from the current
    // pose to the specified pose.
    KDL::Vector r(pose.position.x - current_pose.position.x,
		  pose.position.y - current_pose.position.y,
		  pose.position.z - current_pose.position.z);
    double distance = std::sqrt(r.x()*r.x() + r.y()*r.y() + r.z()*r.z());
    // distance/((seconds)*(frames/second)) = distance/frame.
    // @todo for now, all snap motions execute for 2 s, but this should be 
    // a parameter.
    double dr = distance/(2.0 * getFrameRate());
    double dt = 1/(2.0 * getFrameRate());
    KDL::Vector R((r.x()/distance) * dr,
		  (r.y()/distance) * dr,
		  (r.z()/distance) * dr);

    snap_motion_count_ = 0;
    snap_motion_.clear();

    end_effector_goal_pose_.pose = pose;
    visualization_msgs::InteractiveMarker gripper_marker;
    int_marker_server_->get("r_gripper_marker", gripper_marker);
    gripper_marker.pose = pose;
    tf::Quaternion rot(pose.orientation.x, 
		       pose.orientation.y,
		       pose.orientation.z,
		       pose.orientation.w);
    tf::Quaternion rot2(tf::Vector3(0, 1.0, 0), M_PI/2.0);
    tf::quaternionTFToMsg(rot.inverse() * rot2, gripper_marker.controls.at(0).orientation);

    // ROS_INFO("setting control orientation to (%f, %f, %f, %f)", gripper_marker.controls.at(0).orientation.x,
    // 	     gripper_marker.controls.at(0).orientation.y, gripper_marker.controls.at(0).orientation.z,
    // 	     gripper_marker.controls.at(0).orientation.w);

    int_marker_server_->insert(gripper_marker);
    int_marker_server_->applyChanges();

    sensor_msgs::JointState joint_state;
    joint_state.name.resize(8);
    joint_state.position.resize(8);
    joint_state.name[0] = "r_shoulder_pan_joint";
    joint_state.name[1] = "r_shoulder_lift_joint";
    joint_state.name[2] = "r_upper_arm_roll_joint";
    joint_state.name[3] = "r_elbow_flex_joint";
    joint_state.name[4] = "r_forearm_roll_joint";
    joint_state.name[5] = "r_wrist_flex_joint";
    joint_state.name[6] = "r_wrist_roll_joint";
    joint_state.name[7] = "r_gripper_joint";
    std::vector<double> r_arm_joints(7, 0);
    std::vector<double> r_arm_solution(7, 0);
    for(int i = 7; i < 14; ++i)
    {
      r_arm_joints[i-7] = joint_states_.position[i];
    }

    // For interpolating between orientations (slerp).
    tf::Quaternion current_orientation;
    tf::quaternionMsgToTF(current_pose.orientation, current_orientation);
    tf::Quaternion goal_orientation;
    tf::quaternionMsgToTF(pose.orientation, goal_orientation);

    for(int i = 0; i < static_cast<int>(2.0 * getFrameRate()); ++i)
    {
      // First, find the next pose of the end-effector.
      if(interpolate_position)
      {
	current_pose.position.x += R.x();
	current_pose.position.y += R.y();
	current_pose.position.z += R.z();
      }

      if(interpolate_orientation)
      {
	tf::Quaternion next_orientation = current_orientation.slerp(goal_orientation, dt * (i + 1));
	tf::quaternionTFToMsg(next_orientation, current_pose.orientation);
      }

      // Next, use IK to find the corresponding joint angles.  
      std::vector<double> end_effector_pose(7, 0);
      end_effector_pose[0] = current_pose.position.x;
      end_effector_pose[1] = current_pose.position.y;
      end_effector_pose[2] = current_pose.position.z;
      end_effector_pose[3] = current_pose.orientation.x;
      end_effector_pose[4] = current_pose.orientation.y;
      end_effector_pose[5] = current_pose.orientation.z;
      end_effector_pose[6] = current_pose.orientation.w;

      double roll, pitch, yaw;
      KDL::Rotation rot = KDL::Rotation::Quaternion(current_pose.orientation.x,
						    current_pose.orientation.y,
						    current_pose.orientation.z,
						    current_pose.orientation.w);
      rot.GetRPY(roll, pitch, yaw);
      // ROS_INFO("Pose %d: (%f, %f, %f), (%f, %f, %f).", i+1, 
      // 	       end_effector_pose[0], end_effector_pose[1], end_effector_pose[2],
      // 	       roll, pitch, yaw);

      if(!kdl_robot_model_.computeIK(end_effector_pose, r_arm_joints, r_arm_solution))
      {
	ROS_ERROR("[PR2Sim] IK failed at snap motion interpolation point %d!", i);
	return false;
      }

      for(int j = 0; j < 7; ++j)
      {
	joint_state.position[j] = r_arm_solution[j];
	r_arm_joints[j] = r_arm_solution[j];
      }
      joint_state.position[7] = joint_states_.position[14];
      // ROS_INFO("r_gripper_joint position %d: %f", i, joint_state.position[14]);

      snap_motion_.push_back(joint_state);
    }

    // Also, spend 0.5 s (@todo make this a parameter/make smarter) adjusting to the proper 
    // gripper joint position.
    double delta = (gripper_joint - joint_states_.position[14])/static_cast<int>(0.5 * getFrameRate());
    for(int i = 0; i < static_cast<int>(0.5 * getFrameRate()); ++i)
    {
      joint_state.position[7] += delta;
      // ROS_INFO("r_gripper_joint position %d: %f.", i, joint_state.position[7]);
      snap_motion_.push_back(joint_state);
    }
	  
    ROS_INFO("[PR2Sim] Generated %d points in the interpolation.", int(snap_motion_.size()));

    return true;
  }

  return false;
}

bool PR2Simulator::isSnapDone() const
{
  return (snap_motion_count_ >= int(snap_motion_.size()));
}

void PR2Simulator::processKeyEvent(int key, int type)
{
  if(type == QEvent::KeyPress)
  {
    // @todo For now, we will stop all key press events from being processed if the 
    // simulator is paused. In the future, this may not be desirable.
    if(!isPlaying())
      return;

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
    case Qt::Key_Up:
    {
      moving_gripper_marker_ = true;

      // end_effector_marker_vel_.linear.x = 0;
      // end_effector_marker_vel_.linear.y = 0;
      // end_effector_marker_vel_.linear.z = 0.1;
      end_effector_vel_cmd_.linear.x = 0;
      end_effector_vel_cmd_.linear.y = 0;
      end_effector_vel_cmd_.linear.z = 0.1;

      break;
    }
    case Qt::Key_Down:
    {
      moving_gripper_marker_ = true;

      // end_effector_marker_vel_.linear.x = 0;
      // end_effector_marker_vel_.linear.y = 0;
      // end_effector_marker_vel_.linear.z = -0.1;
      end_effector_vel_cmd_.linear.x = 0;
      end_effector_vel_cmd_.linear.y = 0;
      end_effector_vel_cmd_.linear.z = -0.1;

      break;
    }
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
    case Qt::Key_Up:
    {
      moving_gripper_marker_ = false;

      // end_effector_marker_vel_.linear.x = 0;
      // end_effector_marker_vel_.linear.y = 0;
      // end_effector_marker_vel_.linear.z = 0;
      end_effector_vel_cmd_.linear.x = 0;
      end_effector_vel_cmd_.linear.y = 0;
      end_effector_vel_cmd_.linear.z = 0;

      break;
    }
    case Qt::Key_Down:
    {
      moving_gripper_marker_ = false;

      // end_effector_marker_vel_.linear.x = 0;
      // end_effector_marker_vel_.linear.y = 0;
      // end_effector_marker_vel_.linear.z = 0;
      end_effector_vel_cmd_.linear.x = 0;
      end_effector_vel_cmd_.linear.y = 0;
      end_effector_vel_cmd_.linear.z = 0;

      break;
    }
    default:
      break;
    }
  }
  else
    ROS_ERROR("[PR2Sim] Invalid key event type!");
}

void PR2Simulator::updateEndEffectorMarker()
{
  if((end_effector_controller_.getState() == EndEffectorController::DONE ||
     end_effector_controller_.getState() == EndEffectorController::INVALID_GOAL ||
     recorder_->isReplaying()))
  {
    int_marker_server_->setPose("r_gripper_marker", end_effector_pose_.pose, end_effector_pose_.header);
    int_marker_server_->applyChanges();
  }

  if(end_effector_controller_.getState() == EndEffectorController::DONE)
  {
    if(goal_orientation_changed_)
    {
      snapEndEffectorTo(end_effector_goal_pose_.pose, joint_states_.position[14],
			attached_object_, false, true);
      goal_orientation_changed_ = false;
    }

    end_effector_controller_.setState(EndEffectorController::INITIAL);
  }

  // Get the current pose of the end-effector interactive marker.
  visualization_msgs::InteractiveMarker marker;
  int_marker_server_->get("r_gripper_marker", marker);
  geometry_msgs::Pose pose = marker.pose;

  pose.position.x += (1/getFrameRate())*end_effector_marker_vel_.linear.x;
  pose.position.y += (1/getFrameRate())*end_effector_marker_vel_.linear.y;
  pose.position.z += (1/getFrameRate())*end_effector_marker_vel_.linear.z;

  // Update the pose according to the current velocity command.
  int_marker_server_->setPose("r_gripper_marker", pose, end_effector_pose_.header);
  int_marker_server_->applyChanges();

  // Update the goal pose.
  setEndEffectorGoalPose(pose);
}

void PR2Simulator::updateUpperArmMarker()
{
  // std::vector<double> r_arm_joints(7, 0);
  // for(int i = 7; i < 14; ++i)
  // {
  //   r_arm_joints[i-7] = joint_states_.position[i];
  // }

  // std::vector<double> upper_arm_pose(7, 0);
  // if(!kdl_robot_model_.computeFK(r_arm_joints, "r_upper_arm_roll_link", upper_arm_pose))
  // {
  //   ROS_ERROR("[PR2Sim] Compute FK failed for the upper arm roll link!");
  // }

  // r_upper_arm_roll_pose_.position.x = upper_arm_pose[0];
  // r_upper_arm_roll_pose_.position.y = upper_arm_pose[1];
  // r_upper_arm_roll_pose_.position.z = upper_arm_pose[2];
  // r_upper_arm_roll_pose_.orientation.x = upper_arm_pose[3];
  // r_upper_arm_roll_pose_.orientation.y = upper_arm_pose[4];
  // r_upper_arm_roll_pose_.orientation.z = upper_arm_pose[5];
  // r_upper_arm_roll_pose_.orientation.w = upper_arm_pose[6];

  r_upper_arm_roll_pose_ = robot_markers_.markers.at(4).pose;

  // ROS_INFO("Updating r_upper_arm_roll_link to pose (%f, %f, %f), (%f, %f, %f, %f).",
  // 	   r_upper_arm_roll_pose_.position.x, r_upper_arm_roll_pose_.position.y, r_upper_arm_roll_pose_.position.z,
  // 	   r_upper_arm_roll_pose_.orientation.x, r_upper_arm_roll_pose_.orientation.y, r_upper_arm_roll_pose_.orientation.z,
  // 	   r_upper_arm_roll_pose_.orientation.w);

  KDL::Frame ruarl_in_map(KDL::Rotation::Quaternion(r_upper_arm_roll_pose_.orientation.x,
						    r_upper_arm_roll_pose_.orientation.y,
						    r_upper_arm_roll_pose_.orientation.z,
						    r_upper_arm_roll_pose_.orientation.w),
			  KDL::Vector(r_upper_arm_roll_pose_.position.x,
				      r_upper_arm_roll_pose_.position.y,
				      r_upper_arm_roll_pose_.position.z)
    );

  KDL::Frame marker_in_ruarl(KDL::Rotation::Identity(),
			     KDL::Vector(0.2, 0.0, 0.0));

  KDL::Frame marker_in_map = ruarl_in_map * marker_in_ruarl;
  geometry_msgs::Pose marker_pose;
  marker_pose.position.x = marker_in_map.p.x();
  marker_pose.position.y = marker_in_map.p.y();
  marker_pose.position.z = marker_in_map.p.z();
  double x, y, z, w;
  marker_in_map.M.GetQuaternion(x, y, z, w);
  marker_pose.orientation.x = x;
  marker_pose.orientation.y = y;
  marker_pose.orientation.z = z;
  marker_pose.orientation.w = w;

  // ROS_INFO("updating pose: (%f, %f, %f), (%f, %f, %f, %f)",
  //   marker_pose.position.x, marker_pose.position.y, marker_pose.position.z,
  //   marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z,
  //   marker_pose.orientation.w);

  int_marker_server_->setPose("r_upper_arm_marker", marker_pose);
  int_marker_server_->applyChanges();
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
						     resolveName("map", user_id_),
						     resolveName("base_footprint", user_id_))
    );

  // Broadcast the coordinate frame of the (right) end-effector in the base footprint frame
  // and in the map frame.
  transform.setOrigin(tf::Vector3(end_effector_pose_.pose.position.x,
				  end_effector_pose_.pose.position.y,
				  end_effector_pose_.pose.position.z)
    );
  transform.setRotation(tf::Quaternion(end_effector_pose_.pose.orientation.x,
				       end_effector_pose_.pose.orientation.y,
				       end_effector_pose_.pose.orientation.z,
				       end_effector_pose_.pose.orientation.z)
    );
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform,
						     ros::Time::now(),
						     resolveName("base_footprint", user_id_),
						     resolveName("right_end_effector", user_id_))
    );

  geometry_msgs::Pose end_effector_in_map = getEndEffectorPose();
  transform.setOrigin(tf::Vector3(end_effector_in_map.position.x,
				  end_effector_in_map.position.y,
				  end_effector_in_map.position.z)
    );
  transform.setRotation(tf::Quaternion(end_effector_in_map.orientation.x,
				       end_effector_in_map.orientation.y,
				       end_effector_in_map.orientation.z,
				       end_effector_in_map.orientation.w)
    );
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform,
						     ros::Time::now(),
						     resolveName("map", user_id_),
						     resolveName("right_end_effector", user_id_))
    );

  // Get the pose of the base footprint in the torso lift link frame.
  KDL::Frame base_in_torso_lift_link;
  base_in_torso_lift_link.p.x(0.050);
  base_in_torso_lift_link.p.y(0.0);
  base_in_torso_lift_link.p.z(-0.802 - getTorsoPosition());
  base_in_torso_lift_link.M = KDL::Rotation::Quaternion(0.0, 0.0, 0.0, 1.0);

  // Note that all computed poses of the end-effector are in the base footprint frame.
  kdl_robot_model_.setKinematicsToPlanningTransform(base_in_torso_lift_link.Inverse(), 
						    resolveName("base_footprint", user_id_));
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

bool PR2Simulator::isEndEffectorMoving() const
{
  // @todo THIS DOESN'T WORK.
  if((end_effector_controller_.getState() == EndEffectorController::DONE ||
      end_effector_controller_.getState() == EndEffectorController::INITIAL ||
       end_effector_controller_.getState() == EndEffectorController::INVALID_GOAL) &&
     end_effector_vel_cmd_.linear.x == 0 && end_effector_vel_cmd_.linear.y == 0 
     && end_effector_vel_cmd_.linear.z == 0 && isSnapDone())
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

void PR2Simulator::attach(int id, KDL::Frame transform)
{
  attached_object_ = true;
  attached_id_ = id;
  attached_transform_ = transform;
}

void PR2Simulator::detach()
{
  attached_object_ = false;
}

KDL::Frame PR2Simulator::getAttachedTransform() const
{
  return attached_transform_;
}

bool PR2Simulator::validityCheck(const vector<double>& rangles, 
                                 const vector<double>& langles, 
                                 const BodyPose& bp, 
                                 const geometry_msgs::Pose& object_pose)
{

  if(ignore_collisions_)
    return true;
  
  if(attached_object_){
    //check robot motion
    // ROS_ERROR("Check robot move (attached)");
    if(!object_manager_->checkRobotMove(rangles, langles, bp, attached_id_))
      return false;
    
    // ROS_ERROR("Check object move (attached)");
    if(!object_manager_->checkObjectMove(attached_id_, object_pose, rangles, langles, bp))
      return false;
  }
  else{
    //check robot motion
    // ROS_ERROR("Check robot move");
    if(!object_manager_->checkRobotMove(rangles, langles, bp, -1))
      return false;
  }
  return true;
}

void PR2Simulator::showEndEffectorWorkspaceArc()
{
  visualization_msgs::Marker arc;

  arc.header.stamp = ros::Time::now();
  // arc.header.frame_id = "/map";
  arc.header.frame_id = resolveName("map", user_id_);

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

bool PR2Simulator::getObjectPose(geometry_msgs::Pose &object_pose)
{
  if(!attached_object_)
    return false;

  std::vector<double> end_effector_pose(7, 0);
  end_effector_pose[0] = end_effector_pose_.pose.position.x;
  end_effector_pose[1] = end_effector_pose_.pose.position.y;
  end_effector_pose[2] = end_effector_pose_.pose.position.z;
  end_effector_pose[3] = end_effector_pose_.pose.orientation.x;
  end_effector_pose[4] = end_effector_pose_.pose.orientation.y;
  end_effector_pose[5] = end_effector_pose_.pose.orientation.z;
  end_effector_pose[6] = end_effector_pose_.pose.orientation.w;

  BodyPose body_pose;
  body_pose.x = base_pose_.pose.position.x;
  body_pose.y = base_pose_.pose.position.y;
  body_pose.z = getTorsoPosition();
  body_pose.theta = tf::getYaw(base_pose_.pose.orientation);

  computeObjectPose(end_effector_pose, body_pose, object_pose);

  return true;
}

geometry_msgs::Pose PR2Simulator::getEndEffectorPoseInBase() const
{
  return end_effector_pose_.pose;
}

void PR2Simulator::setMoveEndEffectorWhileDragging(bool move)
{
  move_end_effector_while_dragging_ = move;
}

void PR2Simulator::setMoveBaseWhileDragging(bool move)
{
  move_base_while_dragging_ = move;
}

bool PR2Simulator::closestValidEndEffectorPosition(const geometry_msgs::Pose &current_pose,
						   const std::vector<std::pair<double, double> > &intervals,
						   const std::vector<double> &d,
						   double &x, double &y, double &z,
						   bool verbose)
{
  // @todo for now we assume inputs are correct, but in the future we should check.
  if(verbose)
  {
    ROS_INFO("[PR2Sim] Searching from position (%f, %f, %f) over intervals %f <= x < %f, %f <= y < %f"
	     " and %f <= z < %f with resolutions dx = %f, dy = %f, and dz = %f.",
	     current_pose.position.x, current_pose.position.y, current_pose.position.z,
	     intervals[0].first, intervals[0].second, intervals[1].first, intervals[1].second,
	     intervals[2].first, intervals[2].second, d[0], d[1], d[2]);
  }

  int n = 0;
  double cx = current_pose.position.x;
  double cy = current_pose.position.y;
  double cz = current_pose.position.z;
  std::vector<double> r_arm_joints(7, 0);
  std::vector<double> r_arm_solution(7, 0);
  std::vector<double> candidate_pose(7, 0);
  for(int i = 7; i < 14; ++i)
  {
    r_arm_joints[i-7] = joint_states_.position[i];
  }
  geometry_msgs::Pose candidate;
  candidate.position.x = cx;
  candidate.position.y = cy;
  candidate.position.z = cz;
  candidate.orientation = current_pose.orientation;
  // Continue to search while there still exist valid positions.
  int i_limit = 0;
  int j_limit = 0;
  int k_limit = 0;
  bool i_limit_reached = false;
  bool j_limit_reached = false;
  bool k_limit_reached = false;
  while((candidate.position.x - d[0] >= intervals[0].first && candidate.position.x + d[0] < intervals[0].second) ||
	(candidate.position.y - d[1] >= intervals[1].first && candidate.position.y + d[1] < intervals[1].second) ||
	(candidate.position.z - d[2] >= intervals[2].first && candidate.position.z + d[2] < intervals[2].second))
  {
    // ROS_INFO("n = %d, i_limit = %d, j_limit = %d, k_limit = %d", n, i_limit, j_limit, k_limit);
    for(int i = -i_limit; i <= i_limit; ++i)
    {
      for(int j = -j_limit; j <= j_limit; ++j)
      {
	for(int k = -k_limit; k <= k_limit; ++k)
	{
	  if(std::abs(i) < n && std::abs(j) < n && std::abs(k) < n)
	    continue;

	  // Assign the candidate value to each coordinate if it 
	  // falls within that coordinate's bounds.
	  if((cx + i*d[0]) >= intervals[0].first && (cx + i*d[0]) < intervals[0].second)
	  {
	    candidate.position.x = cx + i*d[0];
	    // ROS_INFO("trying x = %f", candidate.position.x);
	  }
	  else
	    i_limit_reached = true;

	  if((cy + j*d[1]) >= intervals[1].first && (cy + j*d[1]) < intervals[1].second)
	  {
	    candidate.position.y = cy + j*d[1];
	    // ROS_INFO("trying y = %f", candidate.position.y);
	  }
	  else
	    j_limit_reached = true;

	  if((cz + k*d[2]) >= intervals[2].first && (cz + k*d[2]) < intervals[2].second)
	  {
	    candidate.position.z = cz + k*d[2];
	    // ROS_INFO("trying z = %f", candidate.position.z);
	  }
	  else
	    k_limit_reached = true;

	  // ROS_INFO("trying candidate (%f, %f, %f)", candidate.position.x, candidate.position.y,
	  // 	   candidate.position.z);

	  // Test the new candidate position.
	  candidate_pose[0] = candidate.position.x;
	  candidate_pose[1] = candidate.position.y;
	  candidate_pose[2] = candidate.position.z;
	  candidate_pose[3] = candidate.orientation.x;
	  candidate_pose[4] = candidate.orientation.y;
	  candidate_pose[5] = candidate.orientation.z;
	  candidate_pose[6] = candidate.orientation.w;
	  if(kdl_robot_model_.computeIK(candidate_pose, r_arm_joints, r_arm_solution) && n > 0)
	  {
	    if(verbose)
	    {
	      ROS_INFO("Search found a valid IK solution at position (%f, %f, %f) from "
		       "initial position (%f, %f, %f).", candidate.position.x, 
		       candidate.position.y, candidate.position.z, current_pose.position.x, 
		       current_pose.position.y, current_pose.position.z);
	    }

	    x = candidate.position.x;
	    y = candidate.position.y;
	    z = candidate.position.z;
	    return true;
	  }	  
	}
      }
    }

    n++;
    if(!i_limit_reached)
      i_limit = n;
    if(!j_limit_reached)
      j_limit = n;
    if(!k_limit_reached)
      k_limit = n;
  }

  return false;
}

bool PR2Simulator::canMoveRobotMarkers() const
{
  return move_robot_markers_;
}

void PR2Simulator::setMoveRobotMarkers(bool move)
{
  move_robot_markers_ = move;
}

void PR2Simulator::enableOrientationControl()
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker_server_->get("r_gripper_marker", int_marker);

  if(int_marker.controls.size() <= 1)
  {
    visualization_msgs::InteractiveMarkerControl control;

    control.name = "YAW";
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);

    control.name = "ROLL";
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    int_marker.controls.push_back(control);

    control.name = "PITCH";
    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    int_marker.controls.push_back(control);
  }

  int_marker_server_->insert(int_marker);
  int_marker_server_->applyChanges();
}

void PR2Simulator::disableOrientationControl()
{
  visualization_msgs::InteractiveMarker int_marker;
  int_marker_server_->get("r_gripper_marker", int_marker);

  while(int_marker.controls.size() > 1)
  {
    int_marker.controls.pop_back();
  }

  int_marker_server_->insert(int_marker);
  int_marker_server_->applyChanges();
}

void PR2Simulator::enableUpperArmRollControl()
{
  r_upper_arm_roll_pose_ = robot_markers_.markers.at(4).pose;
  
  KDL::Frame ruarl_in_map(KDL::Rotation::Quaternion(r_upper_arm_roll_pose_.orientation.x,
						    r_upper_arm_roll_pose_.orientation.y,
						    r_upper_arm_roll_pose_.orientation.z,
						    r_upper_arm_roll_pose_.orientation.w),
			  KDL::Vector(r_upper_arm_roll_pose_.position.x,
				      r_upper_arm_roll_pose_.position.y,
				      r_upper_arm_roll_pose_.position.z)
    );

  KDL::Frame marker_in_ruarl(KDL::Rotation::Identity(),
			     KDL::Vector(0.20, 0.0, 0.0));

  KDL::Frame marker_in_map = ruarl_in_map * marker_in_ruarl;
  geometry_msgs::Pose marker_pose;
  marker_pose.position.x = marker_in_map.p.x();
  marker_pose.position.y = marker_in_map.p.y();
  marker_pose.position.z = marker_in_map.p.z();
  double x, y, z, w;
  marker_in_map.M.GetQuaternion(x, y, z, w);
  marker_pose.orientation.x = x;
  marker_pose.orientation.y = y;
  marker_pose.orientation.z = z;
  marker_pose.orientation.w = w;

  // ROS_INFO("initial pose: (%f, %f, %f), (%f, %f, %f, %f)",
  //   marker_pose.position.x, marker_pose.position.y, marker_pose.position.z,
  //   marker_pose.orientation.x, marker_pose.orientation.y, marker_pose.orientation.z,
  //   marker_pose.orientation.w);

  // Place an interactive marker control on the right upper arm.
  visualization_msgs::InteractiveMarker int_marker;
  // int_marker.header.frame_id = "/map"; //"/base_footprint";
  int_marker.header.frame_id = resolveName("map", user_id_);
  int_marker.pose = marker_pose; //robot_markers_.markers.at(4).pose;
  int_marker.name = "r_upper_arm_marker";
  int_marker.description = "";
  int_marker.scale = 0.4;

  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.clear();
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.orientation.w = 1;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  int_marker_server_->insert(int_marker,
                             boost::bind(&PR2Simulator::upperArmMarkerFeedback,
			                 this,
			                 _1)
                             );
  int_marker_server_->applyChanges();
}

void PR2Simulator::disableUpperArmRollControl()
{
  int_marker_server_->erase("r_upper_arm_marker");
  int_marker_server_->applyChanges();
}

double PR2Simulator::getTorsoPosition() const
{
  return torso_position_;
}

void PR2Simulator::setTorsoPosition(double position)
{
  // @todo check if it is valid (i.e. within limits).
  torso_position_ = position;
}

void PR2Simulator::setIgnoreCollisions(bool ignore)
{
  ignore_collisions_ = ignore;
}

} // namespace demonstration_visualizer
