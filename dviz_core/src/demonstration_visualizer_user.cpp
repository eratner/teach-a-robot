#include <dviz_core/demonstration_visualizer_user.h>

namespace demonstration_visualizer
{

DemonstrationVisualizerUser::DemonstrationVisualizerUser(int argc, char **argv, int id, bool web)
  : id_(id), ok_(true), web_(web), frame_rate_(10.0), frame_rate_changed_(false)
{
  ROS_INFO("[DVizUser%d] Constructing user.", id_);

  if(!init(argc, argv))
  {
    ROS_ERROR("[DVizUser%d] Unable to connect to ROS master!", id_);
  }

  // For web purposes, we need to fork a process to run an interactive marker proxy node.
  if(web_)
  {
    int rosrun = fork();
    if(rosrun == 0)
    {
      char topic_ns[256];
      sprintf(topic_ns, "topic_ns:=/dviz_user_%d/interactive_markers", id_);
      char target_frame[256];
      sprintf(target_frame, "target_frame:=/dviz_user_%d/map", id_);
      char proxy_name[256];
      sprintf(proxy_name, "__name:=proxy%d", id_);
      execlp("rosrun", "rosrun", "interactive_marker_proxy", "proxy", topic_ns, target_frame, proxy_name, (char *)0);
      ROS_WARN("execlp probably failed in addUser()!");
    }
    else if(rosrun < 0)
    {
      ROS_ERROR("[DVizUser%d] fork failed in constructor!", id_);
    }
  }

  int_marker_server_ = new interactive_markers::InteractiveMarkerServer(resolveName("interactive_markers", id_));
  std::stringstream ss;
  ss << "dviz_user_" << id_;
  pviz_ = new PViz(ss.str());
  recorder_ = new MotionRecorder();

  std::string larm_filename;
  std::string rarm_filename;
  ros::NodeHandle nh("/dviz_core_node");
  nh.param<std::string>("left_arm_description_file", larm_filename, "");
  nh.param<std::string>("right_arm_description_file", rarm_filename, "");
  object_manager_ = new ObjectManager(rarm_filename, larm_filename, id_);
  simulator_ = new PR2Simulator(recorder_, pviz_, int_marker_server_, object_manager_, id_);
  demonstration_scene_manager_ = new DemonstrationSceneManager(pviz_, int_marker_server_, object_manager_, id_);
}

DemonstrationVisualizerUser::~DemonstrationVisualizerUser()
{
  ROS_INFO("[DVizUser%d] Destructing user.", id_);

  if(web_)
  {
    int rosnode = fork();
    if(rosnode == 0)
    {
      char proxy_name[256];
      sprintf(proxy_name, "/proxy%d", id_);
      execlp("rosnode", "rosnode", "kill", proxy_name, (char *)0);
      ROS_WARN("execlp probably failed in destructor!");
    }
    else if(rosnode < 0)
    {
      ROS_ERROR("[DVizUser%d] fork failed in destructor!", id_);
    }
  }

  delete int_marker_server_;
  delete object_manager_;
  delete recorder_;
  delete simulator_;
  delete demonstration_scene_manager_;

  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
}

void DemonstrationVisualizerUser::run()
{
  ROS_INFO("[DVizUser%d] Running at %f frames/sec.", id_, frame_rate_);
  ros::Rate rate(frame_rate_);
  while(ros::ok() && ok_)
  {
    if(frame_rate_changed_)
    {
      rate = ros::Rate(frame_rate_);
      frame_rate_changed_ = false;
    }

    // Run the simulator.
    simulator_->run();

    // Update the demonstration scene.
    demonstration_scene_manager_->updateScene();

    // Update goals and task.
    updateGoalsAndTask();

    ros::spinOnce();
    rate.sleep();
  }
}

bool DemonstrationVisualizerUser::processCommand(dviz_core::Command::Request &req,
						 dviz_core::Command::Response &res)
{
  ROS_INFO("[DVizUser%d] Processing command %s with %d arguments.", id_, req.command.c_str(), req.args.size());

  if(req.command.compare(dviz_core::Command::Request::KILL_USER) == 0)
  {
    ok_ = false;
  } // end KILL_USER
  else if(req.command.compare(dviz_core::Command::Request::PLAY) == 0)
  {
    simulator_->play();
  } // end PLAY
  else if(req.command.compare(dviz_core::Command::Request::PAUSE_NOW) == 0)
  {
    simulator_->pause();
  } // end PAUSE_NOW
  else if(req.command.compare(dviz_core::Command::Request::PAUSE_LATER) == 0)
  {
    simulator_->pauseLater();
  } // end PAUSE_LATER
  else if(req.command.compare(dviz_core::Command::Request::RESET_ROBOT) == 0)
  {
    simulator_->resetRobot();
  } // end RESET_ROBOT
  else if(req.command.compare(dviz_core::Command::Request::LOAD_TASK) == 0)
  {
    if(req.args.size() == 1)
    {
      if(req.args[0].substr(0, 10).compare("package://") == 0)
      {
	int i = req.args[0].substr(10).find("/");
	std::string package = req.args[0].substr(10, i);
	std::string path = req.args[0].substr(10+i);
	std::stringstream ss;
	ss << ros::package::getPath(package) << path;
	demonstration_scene_manager_->loadTask(ss.str());
      }
      else
      {
	demonstration_scene_manager_->loadTask(req.args[0]);
      }
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for load_task (%d given, 1 required).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_task (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end LOAD_TASK
  else if(req.command.compare(dviz_core::Command::Request::LOAD_SCENE) == 0)
  {
    if(req.args.size() == 1)
    {
      if(req.args[0].substr(0, 10).compare("package://") == 0)
      {
	int i = req.args[0].substr(10).find("/");
	std::string package = req.args[0].substr(10, i);
	std::string path = req.args[0].substr(10+i);
	std::stringstream ss;
	ss << ros::package::getPath(package) << path;
	demonstration_scene_manager_->loadScene(ss.str());
      }
      else
      {
	int max_mesh = demonstration_scene_manager_->loadScene(req.args[0]);
	std::stringstream ss; 
	ss << max_mesh;
	res.response = ss.str();
      }
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for load_scene (%d given, 1 required).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_scene (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end LOAD_SCENE
  else if(req.command.compare(dviz_core::Command::Request::SHOW_BASE_PATH) == 0)
  {
    if(req.args.size() == 1)
    {
      if(req.args[0].substr(0, 10).compare("package://") == 0)
      {
	int i = req.args[0].substr(10).find("/");
	std::string package = req.args[0].substr(10, i);
	std::string path = req.args[0].substr(10+i);
	std::stringstream ss;
	ss << ros::package::getPath(package) << path;
	showBasePath(ss.str());
      }
      else
      {
	showBasePath(req.args[0]);
      }
    }
    else if(req.args.size() == 0)
    {
      showBasePath();
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for show_base_path (%d given, 1 optional).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for show_base_path (" << req.args.size() << " given, 1 optional).";
      res.response = ss.str();
      return false;
    }  
  } // end SHOW_BASE_PATH
  else if(req.command.compare(dviz_core::Command::Request::LOAD_MESH) == 0)
  {
    if(req.args.size() == 2 || req.args.size() == 3)
    {
      bool movable = (req.args[1].compare("true") == 0);

      std::string name = "";
      // Get the name of the mesh.
      if(req.args.size() == 3)
      {
	name = req.args[2];
      }
      else
      {
	int i;
	for(i = req.args[0].size()-1; i >= 0; --i)
	{
	  if(req.args[0].substr(i, 1).compare("/") == 0)
	    break;
	}
	name = req.args[0].substr(i);
      }

      ROS_INFO("[DVizUser%d] Adding mesh %s from file %s.", id_, name.c_str(), req.args[0].c_str());
      demonstration_scene_manager_->addMeshFromFile(req.args[0], 20, name, movable);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for load_mesh (%d given, 2 required, 1 optional).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_mesh (" << req.args.size() << " given, 2 required, 1 optional).";
      res.response = ss.str();
      return false;
    }    
  } // end LOAD_MESH
  else if(req.command.compare(dviz_core::Command::Request::BASIC_GRIPPER) == 0)
  {
    if(req.args.size() == 0)
    {
      // Enable basic gripper controls.
      simulator_->enableOrientationControl();
      simulator_->enableUpperArmRollControl();
    }
    else if(req.args.size() == 1)
    {
      bool disable = req.args[0].compare("true") == 0;
      if(disable)
      {
	simulator_->disableOrientationControl();
	simulator_->disableUpperArmRollControl();
      }
      else
      {
	simulator_->enableOrientationControl();
	simulator_->enableUpperArmRollControl();
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for basic_gripper (%d given, 1 optional).",
    		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for basic_gripper (" << req.args.size() << " given, 1 optional).";
      res.response = ss.str();
      return false;
    }    
  } // end BASIC_GRIPPER
  else if(req.command.compare(dviz_core::Command::Request::RESET_TASK) == 0)
  {
    demonstration_scene_manager_->resetTask();
  } // end RESET_TASK
  else if(req.command.compare(dviz_core::Command::Request::BEGIN_RECORDING) == 0)
  {
    if(req.args.size() == 1)
    {
      recorder_->beginRecording(req.args[0]);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for begin_recording (%d given, 1 required).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for begin_recording (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end BEGIN_RECORDING
  else if(req.command.compare(dviz_core::Command::Request::BEGIN_REPLAY) == 0)
  {
    if(req.args.size() == 1)
    {
      recorder_->beginReplay(req.args[0]);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for begin_replay (%d given, 1 required).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for begin_replay (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end BEGIN_REPLAY
  else if(req.command.compare(dviz_core::Command::Request::END_RECORDING) == 0)
  {
    recorder_->endRecording();
  } // end END_RECORDING
  else if(req.command.compare(dviz_core::Command::Request::PROCESS_KEY) == 0)
  {
    if(req.args.size() == 2)
    {
      int key = atoi(req.args[0].c_str());
      int type = atoi(req.args[1].c_str());
      ROS_INFO("[DVizUser%d] Processing key %d, event type %d.", id_, key, type);
      simulator_->processKeyEvent(key, type);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for process_key (%d given, 2 required).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for process_key (" << req.args.size() << " given, 2 required).";
      res.response = ss.str();
      return false;
    }
  } // end PROCESS_KEY
  else if(req.command.compare(dviz_core::Command::Request::SET_BASE_SPEED) == 0)
  {
    if(req.args.size() == 2)
    {
      double linear = atof(req.args[0].c_str());
      double angular = atof(req.args[1].c_str());
      // Speed is given in m/s (or rad/s), must be converted to m/f
      // (or rad/f).
      simulator_->setBaseSpeed(linear == 0 ? 0 : linear * (1/getFrameRate()),
			       angular == 0 ? 0 : angular * (1/getFrameRate()));
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for set_base_speed (%d given, 2 required).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for set_base_speed (" << req.args.size() << " given, 2 required).";
      res.response = ss.str();
      return false;
    }
  } // end SET_BASE_SPEED
  else if(req.command.compare(dviz_core::Command::Request::SET_ARM_SPEED) == 0)
  {
    if(req.args.size() == 1)
    {
      double speed = atof(req.args[0].c_str());
      // Speed is given in m/s, must be converted to m/f.
      simulator_->setEndEffectorSpeed(speed * (1/getFrameRate()));
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for set_arm_speed (%d given, 1 required).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for set_arm_speed (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end SET_ARM_SPEED
  else if(req.command.compare(dviz_core::Command::Request::SET_FRAME_RATE) == 0)
  {
    if(req.args.size() == 1)
    {
      double rate = atof(req.args[0].c_str());
      setFrameRate(rate);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for set_frame_rate (%d given, 1 required).",
		id_, req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for set_frame_rate (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end SET_FRAME_RATE
  else
  {
    ROS_ERROR("[DVizUser%d] Invalid command \"%s\".", id_, req.command.c_str());
    std::stringstream ss;
    ss << "Invalid command \"" << req.command.c_str() << "\".";
    res.response = ss.str();
    return false;
  }

  return true;
}

bool DemonstrationVisualizerUser::processCommand(const std::string &command, 
						 const std::vector<std::string> &args)
{
  dviz_core::Command::Request req;
  req.command = command;
  req.args = args;
  dviz_core::Command::Response res;
  return processCommand(req, res);
}

void DemonstrationVisualizerUser::updateGoalsAndTask()
{
  // Update the task message. 
  std::vector<Goal *> current_goals = demonstration_scene_manager_->getGoals();
  dviz_core::Task task;
  task.current_goal = demonstration_scene_manager_->getCurrentGoal();
  dviz_core::Goal goal;
  std::vector<Goal *>::iterator it;
  for(it = current_goals.begin(); it != current_goals.end(); ++it)
  {
    goal.number = (*it)->getGoalNumber();
    goal.description = (*it)->getDescription();
    goal.type = (*it)->getType();
    switch((*it)->getType())
    {
    case Goal::PICK_UP:
    {
      PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(*it);
      goal.object_id = pick_up_goal->getObjectID();
      goal.grasp_pose = pick_up_goal->getGraspPose();
      goal.initial_object_pose = pick_up_goal->getInitialObjectPose();
      goal.grasp_distance = pick_up_goal->getGraspDistance();
      goal.gripper_joint_position = pick_up_goal->getGripperJointPosition();

      break;
    }
    case Goal::PLACE:
    {
      PlaceGoal *place_goal = static_cast<PlaceGoal *>(*it);
      goal.object_id = place_goal->getObjectID();
      goal.ignore_yaw = place_goal->ignoreYaw();
      goal.place_pose = place_goal->getPlacePose();

      break;
    }
    default:
      break;
    }
    task.goals.push_back(goal);
  }
  task_pub_.publish(task);

  // Check to see if the end-effector has reached a goal.
  if(demonstration_scene_manager_->editGoalsMode() ||
     demonstration_scene_manager_->taskDone() ||
     demonstration_scene_manager_->getNumGoals() == 0)
    return;

  Goal *current_goal = demonstration_scene_manager_->getGoal(demonstration_scene_manager_->getCurrentGoal());

  switch(current_goal->getType())
  {
  case Goal::PICK_UP:
  {
    PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(current_goal);
    bool goal_reachable = true;
      
    if(demonstration_scene_manager_->hasReachedGoal(demonstration_scene_manager_->getCurrentGoal(), 
						    simulator_->getEndEffectorPose(), 0.05) 
       && !simulator_->isBaseMoving() /*&& !simulator_->isEndEffectorMoving()*/)
    {
      ROS_INFO("[DVizUser%d] Reached goal %d!", id_, demonstration_scene_manager_->getCurrentGoal());

      geometry_msgs::Pose object_pose = demonstration_scene_manager_->getObjectPose(pick_up_goal->getObjectID());
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
      ROS_INFO("[DVizUser%d] Current end-effector RPY: (%f, %f, %f).", id_, current_roll, current_pitch, current_yaw);
								
      double roll, pitch, yaw;
      gripper_in_base.M.GetRPY(roll, pitch, yaw);
      ROS_INFO("[DVizUser%d] Goal reached, snapping end-effector to grasp pose (%f, %f, %f), (%f, %f, %f).", id_,
	       goal_gripper_pose.position.x, goal_gripper_pose.position.y, goal_gripper_pose.position.z, 
	       roll, pitch, yaw);

      double dist_A = angles::normalize_angle_positive(roll) - angles::normalize_angle_positive(current_roll);
      double dist_B = angles::normalize_angle_positive(roll + M_PI) - angles::normalize_angle_positive(current_roll);
      if(std::abs(dist_A) > std::abs(dist_B))
      {
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
	  
	// Q_EMIT goalComplete(getSceneManager()->getCurrentGoal());

	demonstration_scene_manager_->setCurrentGoal(demonstration_scene_manager_->getCurrentGoal() + 1);
      }
      else
      {
	ROS_ERROR("[DVizUser%d] Unable to reach goal %d!", id_, demonstration_scene_manager_->getCurrentGoal());
      }
    }

    break;
  }
  case Goal::PLACE:
  {
    PlaceGoal *place_goal = static_cast<PlaceGoal *>(current_goal);
    geometry_msgs::Pose object_pose = object_manager_->getMarker(place_goal->getObjectID()).pose;
    bool goal_reachable = true;

    if(demonstration_scene_manager_->hasReachedGoal(demonstration_scene_manager_->getCurrentGoal(), 
						    object_pose, 0.08) 
       && !simulator_->isBaseMoving() /*&& !simulator_->isEndEffectorMoving()*/)
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
	  ROS_ERROR("[DVizUser] No attached object!");
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

      // @todo set the correct gripper joint position (2nd argument).
      goal_reachable = simulator_->snapEndEffectorTo(goal_gripper_pose,
						     EndEffectorController::GRIPPER_OPEN_ANGLE,
						     true);

      if(goal_reachable)
      {
	simulator_->detach();
	  
	// Q_EMIT goalComplete(getSceneManager()->getCurrentGoal());

	demonstration_scene_manager_->setCurrentGoal(demonstration_scene_manager_->getCurrentGoal() + 1);
      }
      else
      {
	ROS_ERROR("[DVizUser%d] Unable to reach goal %d!", id_, demonstration_scene_manager_->getCurrentGoal());
      }
    }
	
    break;
  }
  default:
    break;
  }
}

void DemonstrationVisualizerUser::showBasePath(const std::string &filename)
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
  
  // @todo no marker pub!!!
  // marker_pub_.publish(base_path);
}

DemonstrationSceneManager *DemonstrationVisualizerUser::getSceneManager()
{
  return demonstration_scene_manager_;
}

bool DemonstrationVisualizerUser::init(int argc, char **argv)
{
  std::stringstream ss;
  ss << "dviz_user_" << id_;
  ros::init(argc, argv, ss.str());
  sleep(2.0);

  // Make sure that we can communicate with the master.
  if(!ros::master::check())
    return false;

  ros::start();

  ros::NodeHandle nh("~");
  command_service_ = nh.advertiseService<dviz_core::Command::Request,
					 dviz_core::Command::Response>("dviz_command",
					 boost::bind(static_cast<bool (DemonstrationVisualizerUser::*)
						     (dviz_core::Command::Request &, 
						      dviz_core::Command::Response &)>(
							&DemonstrationVisualizerUser::processCommand),
						     this, _1, _2));

  // Advertise a topic for publishing the current task.
  task_pub_ = nh.advertise<dviz_core::Task>("dviz_task", 1);

  return true;
}

void DemonstrationVisualizerUser::setFrameRate(double rate)
{
  frame_rate_changed_ = true;
  simulator_->setFrameRate(rate);
  frame_rate_ = rate;
}

double DemonstrationVisualizerUser::getFrameRate() const
{
  return frame_rate_;
}

} // namespace demonstration_visualizer
