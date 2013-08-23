#include <dviz_core/demonstration_visualizer.h>

namespace demonstration_visualizer {

  DemonstrationVisualizerCore::DemonstrationVisualizerCore(int argc, char **argv, bool threaded)
{
  if(!init(argc, argv))
    ROS_ERROR("[DVizCore] Unable to connect to master!");

  recorder_ = new MotionRecorder();

  pviz_ = new PViz();

  int_marker_server_ = new interactive_markers::InteractiveMarkerServer("dviz_interactive_markers");

  std::string larm_filename;
  std::string rarm_filename;
  ros::NodeHandle nh("~");
  nh.param<std::string>("left_arm_description_file", larm_filename, "");
  nh.param<std::string>("right_arm_description_file", rarm_filename, "");
  object_manager_ = new ObjectManager(rarm_filename, larm_filename);

  demonstration_scene_manager_ = new DemonstrationSceneManager(pviz_, int_marker_server_, object_manager_);

  simulator_ = new PR2Simulator(recorder_, pviz_, int_marker_server_, object_manager_);
  
  if(threaded)
  {
    // Start the thread.
    start();
  }
}

DemonstrationVisualizerCore::~DemonstrationVisualizerCore()
{
  delete simulator_;
  delete demonstration_scene_manager_;
  delete object_manager_;
  delete recorder_;
  delete pviz_;

  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

bool DemonstrationVisualizerCore::init(int argc, char **argv)
{
  ros::init(argc, argv, "demonstration_visualizer");
  sleep(2.0);

  // Make sure that we can communicate with the master.
  if(!ros::master::check())
    return false;

  ros::start();

  ros::NodeHandle nh("~");

  nh.param("world_frame", world_frame_, std::string("/map"));

  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

  // Advertise topic for publishing end-effector velocity commands.
  end_effector_vel_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/end_effector_vel_cmd", 1);

  base_vel_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/vel_cmd", 1);

  camera_update_pub_ = nh.advertise<dviz_core::CameraUpdate>(
    "/dviz_camera_update", 1);

  return true;
}

std::string DemonstrationVisualizerCore::getWorldFrame() const
{
  return world_frame_;
}

void DemonstrationVisualizerCore::pauseSimulator()
{
  simulator_->pause();
}

void DemonstrationVisualizerCore::pauseSimulatorLater()
{
  simulator_->pauseLater();
}

void DemonstrationVisualizerCore::playSimulator()
{
  simulator_->play();
}

void DemonstrationVisualizerCore::run()
{
  ros::Rate rate(10.0);
  while(ros::ok())
  {
    ROS_DEBUG("[dvn] Running simulator.");
    simulator_->run();

    getSceneManager()->updateScene();

    updateGoals();

    // Focus the camera according to the position of the end-effector and the 
    // current goal.
    if(!getSceneManager()->taskDone() && getSceneManager()->getNumGoals() > 0)
    {
      dviz_core::CameraUpdate camera_update;
      camera_update.base_pose = getBasePose();
      camera_update.end_effector_pose = getEndEffectorPose();
      camera_update.object_pose = getSceneManager()->getCurrentGoalPose();
      camera_update_pub_.publish(camera_update);

      Q_EMIT updateCamera(getEndEffectorPose(), getSceneManager()->getCurrentGoalPose());
    }
    else
    {
      dviz_core::CameraUpdate camera_update;
      camera_update.base_pose = getBasePose();
      camera_update.end_effector_pose = getEndEffectorPose();
      // @todo sort of a hack, should make this cleaner.
      camera_update.object_pose = getEndEffectorPose();
      camera_update_pub_.publish(camera_update);      

      Q_EMIT updateCamera(getEndEffectorPose(), getEndEffectorPose());
    }

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("[DVizCore] Shutting down ROS...");
  Q_EMIT rosShutdown();
}

void DemonstrationVisualizerCore::updateGoals()
{
  // Check to see if the end-effector has reached a goal.
  if(getSceneManager()->editGoalsMode() ||
     getSceneManager()->taskDone() ||
     getSceneManager()->getNumGoals() == 0)
    return;

  Goal *current_goal = getSceneManager()->getGoal(getSceneManager()->getCurrentGoal());

  switch(current_goal->getType())
  {
  case Goal::PICK_UP:
  {
    PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(current_goal);
    bool goal_reachable = true;
      
    if(getSceneManager()->hasReachedGoal(getSceneManager()->getCurrentGoal(), getEndEffectorPose(), 0.05) &&
      !simulator_->isBaseMoving() /*&& !simulator_->isEndEffectorMoving()*/)
    {
      ROS_INFO("[DVizCore] Reached goal %d!", getSceneManager()->getCurrentGoal());

      geometry_msgs::Pose object_pose = getSceneManager()->getObjectPose(pick_up_goal->getObjectID());
      geometry_msgs::Pose gripper_pose = pick_up_goal->getGraspPose();
      KDL::Frame object_in_map(KDL::Rotation::Quaternion(object_pose.orientation.x,
							 object_pose.orientation.y,
							 object_pose.orientation.z,
							 object_pose.orientation.w),
			       KDL::Vector(object_pose.position.x,
					   object_pose.position.y,
					   object_pose.position.z)
	);
      KDL::Frame marker_in_map(KDL::Rotation::Quaternion(gripper_pose.orientation.x,
							 gripper_pose.orientation.y,
							 gripper_pose.orientation.z,
							 gripper_pose.orientation.w),
			       KDL::Vector(gripper_pose.position.x,
					   gripper_pose.position.y,
					   gripper_pose.position.z)
	);
      KDL::Frame gripper_in_marker(KDL::Rotation::Identity(),
				   KDL::Vector(-1.0*pick_up_goal->getGraspDistance(),
					       0.0,
					       0.0)
	);
      KDL::Frame gripper_in_map = marker_in_map * gripper_in_marker;

      KDL::Frame object_in_gripper = gripper_in_map.Inverse() * object_in_map;
	    
      // Get the pose of the gripper in the base frame.
      geometry_msgs::Pose base_pose = simulator_->getBasePose();
      KDL::Frame base_in_map(KDL::Rotation::Quaternion(base_pose.orientation.x,
						       base_pose.orientation.y,
						       base_pose.orientation.z,
						       base_pose.orientation.w),
			     KDL::Vector(base_pose.position.x,
					 base_pose.position.y,
					 base_pose.position.z)
	);

      KDL::Frame gripper_in_base = base_in_map.Inverse() * gripper_in_map;
	  
      geometry_msgs::Pose goal_gripper_pose;
      goal_gripper_pose.position.x = gripper_in_base.p.x();
      goal_gripper_pose.position.y = gripper_in_base.p.y();
      goal_gripper_pose.position.z = gripper_in_base.p.z();
      double x, y, z, w;
      gripper_in_base.M.GetQuaternion(x, y, z, w);
      goal_gripper_pose.orientation.x = x;
      goal_gripper_pose.orientation.y = y;
      goal_gripper_pose.orientation.z = z;
      goal_gripper_pose.orientation.w = w;

      geometry_msgs::Pose end_effector_pose = simulator_->getEndEffectorPoseInBase();
      KDL::Frame current_gripper_in_base(KDL::Rotation::Quaternion(end_effector_pose.orientation.x,
								   end_effector_pose.orientation.y,
								   end_effector_pose.orientation.z,
								   end_effector_pose.orientation.w),
					 KDL::Vector(end_effector_pose.position.x,
						     end_effector_pose.position.y,
						     end_effector_pose.position.z)
	);

      double current_roll, current_pitch, current_yaw;
      current_gripper_in_base.M.GetRPY(current_roll, current_pitch, current_yaw);
      ROS_INFO("[DVizCore] Current end-effector RPY: (%f, %f, %f).", current_roll, current_pitch, current_yaw);
								
      double roll, pitch, yaw;
      gripper_in_base.M.GetRPY(roll, pitch, yaw);
      ROS_INFO("[DVizCore] Goal reached, snapping end-effector to grasp pose (%f, %f, %f), (%f, %f, %f).",
	       goal_gripper_pose.position.x, goal_gripper_pose.position.y, goal_gripper_pose.position.z, 
	       roll, pitch, yaw);

      double dist_A = angles::normalize_angle_positive(roll) - angles::normalize_angle_positive(current_roll);
      double dist_B = angles::normalize_angle_positive(roll + M_PI) - angles::normalize_angle_positive(current_roll);
      ROS_INFO("dist_A = %f, dist_B = %f", dist_A, dist_B);
      if(std::abs(dist_A) > std::abs(dist_B))
      {
	ROS_INFO("Choosing the other symmetric gripper roll.");
	KDL::Rotation rot = KDL::Rotation::RPY(roll + M_PI, pitch, yaw);
	rot.GetQuaternion(x, y, z, w);
	goal_gripper_pose.orientation.x = x;
	goal_gripper_pose.orientation.y = y;
	goal_gripper_pose.orientation.z = z;
	goal_gripper_pose.orientation.w = w;

	marker_in_map.M.GetRPY(roll, pitch, yaw);
	marker_in_map.M = KDL::Rotation::RPY(roll + M_PI, pitch, yaw);
	gripper_in_map = marker_in_map * gripper_in_marker;
	object_in_gripper = gripper_in_map.Inverse() * object_in_map;
      }

      goal_reachable = simulator_->snapEndEffectorTo(goal_gripper_pose,
						     pick_up_goal->getGripperJointPosition(),
						     false);

      if(goal_reachable)
      {
	simulator_->attach(pick_up_goal->getObjectID(), object_in_gripper);
	  
	Q_EMIT goalComplete(getSceneManager()->getCurrentGoal());

	getSceneManager()->setCurrentGoal(getSceneManager()->getCurrentGoal() + 1);
      }
      else
      {
	ROS_ERROR("Unable to reach goal %d!", getSceneManager()->getCurrentGoal());
      }
    }

    break;
  }
  case Goal::PLACE:
  {
    PlaceGoal *place_goal = static_cast<PlaceGoal *>(current_goal);
    geometry_msgs::Pose object_pose = object_manager_->getMarker(place_goal->getObjectID()).pose;
    bool goal_reachable = true;

    if(getSceneManager()->hasReachedGoal(getSceneManager()->getCurrentGoal(), object_pose, 0.08) &&
      !simulator_->isBaseMoving() /*&& !simulator_->isEndEffectorMoving()*/)
    {
      // Snap the gripper to the correct position so that the object that it is holding moves 
      // smoothly to the goal pose. 
      geometry_msgs::Pose object_goal_pose = place_goal->getPlacePose();
      if(place_goal->ignoreYaw())
      {
	ROS_INFO("Ignoring yaw...");
	geometry_msgs::Pose object_pose;
	if(!simulator_->getObjectPose(object_pose))
	{
	  ROS_ERROR("[DVizCore] No attached object!");
	  return;
	}

	double goal_yaw = tf::getYaw(object_pose.orientation);
	KDL::Rotation rot = KDL::Rotation::Quaternion(object_goal_pose.orientation.x,
						      object_goal_pose.orientation.y,
						      object_goal_pose.orientation.z,
						      object_goal_pose.orientation.w);
	double roll, pitch, yaw;
	rot.GetRPY(roll, pitch, yaw);
	tf::Quaternion goal_orientation;
	goal_orientation.setRPY(roll, pitch, goal_yaw);
	tf::quaternionTFToMsg(goal_orientation, object_goal_pose.orientation);
      }

      KDL::Frame object_in_map(KDL::Rotation::Quaternion(object_goal_pose.orientation.x,
							 object_goal_pose.orientation.y,
							 object_goal_pose.orientation.z,
							 object_goal_pose.orientation.w),
			       KDL::Vector(object_goal_pose.position.x,
					   object_goal_pose.position.y,
					   object_goal_pose.position.z));

      KDL::Frame object_in_gripper = simulator_->getAttachedTransform();

      KDL::Frame gripper_in_map = object_in_map * object_in_gripper.Inverse();

      geometry_msgs::Pose base_pose = simulator_->getBasePose();

      KDL::Frame base_in_map(KDL::Rotation::Quaternion(base_pose.orientation.x,
						       base_pose.orientation.y,
						       base_pose.orientation.z,
						       base_pose.orientation.w),
			     KDL::Vector(base_pose.position.x,
					 base_pose.position.y,
					 base_pose.position.z));

      KDL::Frame gripper_in_base = base_in_map.Inverse() * gripper_in_map;
      geometry_msgs::Pose goal_gripper_pose;
      goal_gripper_pose.position.x = gripper_in_base.p.x();
      goal_gripper_pose.position.y = gripper_in_base.p.y();
      goal_gripper_pose.position.z = gripper_in_base.p.z();
      double x, y, z, w;
      gripper_in_base.M.GetQuaternion(x, y, z, w);
      goal_gripper_pose.orientation.x = x;
      goal_gripper_pose.orientation.y = y;
      goal_gripper_pose.orientation.z = z;
      goal_gripper_pose.orientation.w = w;

      ROS_INFO("current gripper position = (%f, %f, %f); goal gripper position = (%f, %f, %f)",
	       getEndEffectorPose().position.x, getEndEffectorPose().position.y, getEndEffectorPose().position.z,
	       goal_gripper_pose.position.x, goal_gripper_pose.position.y, goal_gripper_pose.position.z);

      // @todo set the correct gripper joint position (2nd argument).
      goal_reachable = simulator_->snapEndEffectorTo(goal_gripper_pose,
						     EndEffectorController::GRIPPER_OPEN_ANGLE,
						     true);

      if(goal_reachable)
      {
	simulator_->detach();
	  
	Q_EMIT goalComplete(getSceneManager()->getCurrentGoal());

	getSceneManager()->setCurrentGoal(getSceneManager()->getCurrentGoal() + 1);
      }
      else
      {
	ROS_ERROR("[DVizCore] Unable to reach goal %d!", getSceneManager()->getCurrentGoal());
      }

      // simulator_->detach();

      // Snap the object to the place pose.
      // object_manager_->moveObject(place_goal->getObjectID(), place_goal->getPlacePose());

      // Reset the orientation of the gripper.
      // ROS_INFO("Resetting right gripper orientation...");
      // simulator_->resetGripperOrientation();
	
      // Q_EMIT goalComplete(getSceneManager()->getCurrentGoal());

      // getSceneManager()->setCurrentGoal(getSceneManager()->getCurrentGoal() + 1);
    }
	
    break;
  }
  default:
    break;
  }
}

void DemonstrationVisualizerCore::setRobotSpeed(double linear, double angular)
{
  simulator_->setSpeed(linear, angular);
}

void DemonstrationVisualizerCore::resetRobot()
{
  // simulator_->resetRobot();
  simulator_->resetRobotTo(getSceneManager()->getInitialRobotPose(),
                           getSceneManager()->getInitialTorsoPosition());
}

DemonstrationSceneManager *DemonstrationVisualizerCore::getSceneManager()
{
  return demonstration_scene_manager_;
}

MotionRecorder *DemonstrationVisualizerCore::getMotionRecorder()
{
  return recorder_;
}

interactive_markers::InteractiveMarkerServer *DemonstrationVisualizerCore::getInteractiveMarkerServer()
{
  return int_marker_server_;
}

void DemonstrationVisualizerCore::processKeyEvent(int key, int type)
{
  // Pass along the key event to the simulator.
  simulator_->processKeyEvent(key, type);
}

void DemonstrationVisualizerCore::prepGripperForGoal(int goal_number)
{
  ROS_INFO("[DVizCore] Prepping gripper for goal %d.", goal_number);

  Goal *goal = getSceneManager()->getGoal(goal_number);
  if(goal->getType() == Goal::PICK_UP)
  {
    PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(goal);

    geometry_msgs::Pose grasp_pose = pick_up_goal->getGraspPose();
    geometry_msgs::Pose current_pose = getEndEffectorPoseInBase();
    geometry_msgs::Pose goal_pose = current_pose;

    KDL::Rotation grasp_orientation = KDL::Rotation::Quaternion(grasp_pose.orientation.x,
								grasp_pose.orientation.y,
								grasp_pose.orientation.z,
								grasp_pose.orientation.w);
    double r, p, y;
    grasp_orientation.GetRPY(r, p, y);
    tf::Quaternion goal_orientation;
    goal_orientation.setRPY(r, p, 0.0);

    tf::quaternionTFToMsg(goal_orientation, goal_pose.orientation);

    simulator_->snapEndEffectorTo(goal_pose, EndEffectorController::GRIPPER_OPEN_ANGLE,
                                  false, false, true);
  }
  else
  {
    ROS_ERROR("[DVizCore] Goal %d is not a pick up goal!", goal_number);
  }
}

void DemonstrationVisualizerCore::setGripperOrientationControl(bool enabled)
{
  if(enabled)
  {
    simulator_->enableOrientationControl();
    simulator_->enableUpperArmRollControl();
  }
  else
  {
    simulator_->disableOrientationControl();
    simulator_->disableUpperArmRollControl();
  }
}

void DemonstrationVisualizerCore::setIgnoreCollisions(bool ignore)
{
  simulator_->setIgnoreCollisions(ignore);
}

geometry_msgs::Pose DemonstrationVisualizerCore::getBasePose()
{
  return simulator_->getBasePose();
}

geometry_msgs::Pose DemonstrationVisualizerCore::getEndEffectorPose()
{
  return simulator_->getEndEffectorPose();
}

geometry_msgs::Pose DemonstrationVisualizerCore::getEndEffectorPoseInBase()
{
  return simulator_->getEndEffectorPoseInBase();
}

geometry_msgs::Pose DemonstrationVisualizerCore::getEndEffectorMarkerPose()
{
  return simulator_->getEndEffectorMarkerPose();
}

void DemonstrationVisualizerCore::setBaseCommand(const geometry_msgs::Pose &pose)
{
  simulator_->setRobotBaseCommand(pose);
}

bool DemonstrationVisualizerCore::setEndEffectorGoalPose(const geometry_msgs::Pose &goal_pose)
{
  return simulator_->setEndEffectorGoalPose(goal_pose);
}

void DemonstrationVisualizerCore::setBaseVelocity(const geometry_msgs::Twist &velocity)
{
  simulator_->updateBaseVelocity(velocity);
}

void DemonstrationVisualizerCore::setJointStates(const sensor_msgs::JointState &joints)
{
  simulator_->setJointStates(joints);
}

void DemonstrationVisualizerCore::showBasePath(const std::string &filename)
{
  visualization_msgs::Marker base_path;

  if(filename.empty())
  {
    base_path = recorder_->getBasePath();
  }
  else
  {
    if(!recorder_->getBasePath(filename, base_path))
    {
      ROS_ERROR("[DVizCore] Error getting the base path!");
      return;
    }
  }
  
  marker_pub_.publish(base_path);
}

void DemonstrationVisualizerCore::showInteractiveGripper(int goal_number)
{
  PickUpGoal *goal = static_cast<PickUpGoal *>(getSceneManager()->getGoal(goal_number));

  geometry_msgs::Pose gripper_pose = goal->getGraspPose();

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/map";
  std::stringstream s;
  s << "grasp_marker_goal_" << goal_number;
  int_marker.name = s.str();
  int_marker.description = "";
  int_marker.pose = gripper_pose;

  visualization_msgs::InteractiveMarkerControl control;
  std::vector<visualization_msgs::Marker> markers;
  geometry_msgs::Pose origin; 
  // @todo compute this based on the goal object.
  origin.position.x = -(goal->getGraspDistance());
  origin.position.y = origin.position.z = 0;
  origin.orientation.x = origin.orientation.y = origin.orientation.z = 0;
  origin.orientation.w = 1;
  pviz_->getGripperMeshesMarkerMsg(origin, 0.2, "pr2_simple_sim", 1, goal->getGripperJointPosition(), markers);

  for(int i = 0; i < int(markers.size()); ++i)
  {
    markers.at(i).header.frame_id = "";
    control.markers.push_back(markers.at(i));
  }
  control.always_visible = true;
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  int_marker.controls.push_back(control);

  int_marker_server_->insert(int_marker,
			     boost::bind(
			       &DemonstrationVisualizerCore::graspMarkerFeedback,
			       this,
			       _1)
			     );
  int_marker_server_->applyChanges();
}

void DemonstrationVisualizerCore::graspMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  // ROS_INFO("[DVizCore] Moving grasp interactive marker %s.", feedback->marker_name.c_str());

  int i = feedback->marker_name.size()-1;
  for(; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  getSceneManager()->setGraspPose(atoi(feedback->marker_name.substr(i+1).c_str()),
				     feedback->pose);
}

void DemonstrationVisualizerCore::disableRobotMarkerControl()
{
  simulator_->setMoveRobotMarkers(false);
}

void DemonstrationVisualizerCore::enableRobotMarkerControl()
{
  simulator_->setMoveRobotMarkers(true);
}

} // namespace demonstration_visualizer
