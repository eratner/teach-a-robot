#include <dviz_core/demonstration_visualizer_user.h>

namespace demonstration_visualizer
{

DemonstrationVisualizerUser::DemonstrationVisualizerUser(int argc, char **argv, int id, bool web)
  : id_(id), ok_(true), web_(web), frame_rate_(10.0), frame_rate_changed_(false), accepted_grasp_(true), ping_count_(0), ping_delay_count_(0)
{
  ROS_INFO("[DVizUser%d] Constructing user", id_);

  if(!init(argc, argv))
  {
    ROS_ERROR("[DVizUser%d] Unable to connect to ROS master!", id_);
  }

  // For web purposes, we need to fork a process to run an interactive marker proxy node
  if(web_)
  {
    // Fork a new process for the interactive_marker_proxy
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
      ROS_WARN("execlp probably failed to spawn interactive marker proxy in addUser()!");
    }
    else if(rosrun < 0)
    {
      ROS_ERROR("[DVizUser%d] fork for interactive marker proxy failed in constructor!", id_);
    }
    // Fork a new process for the tf2_web_republisher
    rosrun = fork();
    if(rosrun == 0)
    {
      char node_name[256];
      sprintf(node_name, "__ns:=dviz_user_%d", id_);
      execlp("rosrun", "rosrun", "tf2_web_republisher", "tf2_web_republisher", node_name, (char *)0);
      ROS_WARN("execlp probably failed to spawn tf2 web republisher in addUser()!");
    }
    else if(rosrun < 0)
    {
      ROS_ERROR("[DVizUser%d] fork for interactive marker proxy failed in constructor!", id_);
    }
  }

  int_marker_server_ = new interactive_markers::InteractiveMarkerServer(resolveName("interactive_markers", id_));
  std::stringstream ss;
  ss << "dviz_user_" << id_;
  pviz_ = new PViz(ss.str());
  pviz_->setReferenceFrame(resolveName("map", id_));
  recorder_ = new MotionRecorder(id_);
  ros::NodeHandle handle;
  ss.str(std::string());
  ss << "/dviz_user_" << id_ << "/visualization_marker_array";
  marker_array_pub_ = handle.advertise<visualization_msgs::MarkerArray>(ss.str(), 500);
  ss.str(std::string());
  ss << "/dviz_user_" << id_ << "/ping";
  ping_pub_ = handle.advertise<std_msgs::Empty>(ss.str(), 30);
  ss.str(std::string());
  ss << "/dviz_user_" << id_ << "/ping_response";
  ping_sub_ = handle.subscribe(ss.str(), 1,
			       &DemonstrationVisualizerUser::pingResponse,
			       this);

  core_command_client_ = handle.serviceClient<dviz_core::Command>("/dviz_command");

  std::string larm_filename;
  std::string rarm_filename;
  ros::NodeHandle nh("/dviz_core_node");
  std::string default_larm_filename = ros::package::getPath("pr2_collision_checker") + "/config/pr2_left_arm.cfg";
  std::string default_rarm_filename = ros::package::getPath("pr2_collision_checker") + "/config/pr2_right_arm.cfg";
  nh.param<std::string>("left_arm_description_file", larm_filename, default_larm_filename);
  nh.param<std::string>("right_arm_description_file", rarm_filename, default_rarm_filename);
  object_manager_ = new ObjectManager(rarm_filename, larm_filename, id_);
  simulator_ = new PR2Simulator(recorder_, pviz_, int_marker_server_, object_manager_, id_);
  demonstration_scene_manager_ = new DemonstrationSceneManager(pviz_, int_marker_server_, object_manager_, id_);
  ProcessInfo::initCPU();
}

DemonstrationVisualizerUser::~DemonstrationVisualizerUser()
{
  ROS_INFO("[DVizUser%d] Destructing user", id_);

  dviz_core::Command decr_command;
  decr_command.request.command = dviz_core::Command::Request::DECR_USERS;
  if(!core_command_client_.call(decr_command))
  {
    ROS_ERROR("[DVizUser%d] Failed to decr users!", id_);
  }

  if(web_)
  {
    // Kill the interactive_marker_proxy process
    int rosnode = fork();
    if(rosnode == 0)
    {
      char proxy_name[256];
      sprintf(proxy_name, "/proxy%d", id_);
      execlp("rosnode", "rosnode", "kill", proxy_name, (char *)0);
      ROS_WARN("execlp probably failed for interactive marker proxy in destructor!");
    }
    else if(rosnode < 0)
    {
      ROS_ERROR("[DVizUser%d] fork failed for interactive marker proxy in destructor!", id_);
    }
    // Kill the tf2_web_republisher process
    rosnode = fork();
    if(rosnode == 0)
    {
      char node_name[256];
      sprintf(node_name, "/dviz_user_%d/tf2_web_republisher", id_);
      execlp("rosnode", "rosnode", "kill", node_name, (char *)0);
      ROS_WARN("execlp probably failed for tf2 web republisher in destructor!");
    }
    else if(rosnode < 0)
    {
      ROS_ERROR("[DVizUser%d] fork failed for tf2 web republisher in destructor!", id_);
    }
  }

  recorder_->endRecording();

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
  ROS_INFO("[DVizUser%d] Running at %f frames/sec", id_, frame_rate_);
  ros::Rate rate(frame_rate_);
  while(ros::ok() && ok_)
  {
    if(frame_rate_changed_)
    {
      rate = ros::Rate(frame_rate_);
      frame_rate_changed_ = false;
    }

    // Run the simulator
    simulator_->run();

    // Update the demonstration scene
    demonstration_scene_manager_->updateScene();

    // Update goals and task
    updateGoalsAndTask();

    if(web_ && ping_count_ > 4)
    {
      ROS_ERROR("[DVizUser%d] DVizClient has timed out, destroying user", id_);
      ok_ = false;
      // Call kill user so that the core can update the user count
      // dviz_core::Command kill_command;
      // kill_command.request.command = dviz_core::Command::Request::KILL_USER;
      // std::stringstream ss;
      // ss << id_;
      // kill_command.request.args.push_back(ss.str());
      // if(!core_command_client_.call(kill_command))
      // {
      // 	ROS_ERROR("[DVizUser%d] Failed to kill self!", id_);
      // }
    }

    if(web_ && ping_delay_count_ >= 300)
    {
      std_msgs::Empty ping;
      ping_pub_.publish(ping);
      ping_count_++;
      if(ping_count_ >= 2)
	ROS_WARN("[DVizUser%d] Ping count raised to %d", id_, ping_count_);

      ping_delay_count_ = 0;
    }

    ping_delay_count_++;

    ros::spinOnce();
    rate.sleep();
  }
}

bool DemonstrationVisualizerUser::processCommand(dviz_core::Command::Request &req,
						 dviz_core::Command::Response &res)
{
  //ROS_INFO("[DVizUser%d] Processing command %s with %d arguments.", id_, req.command.c_str(), req.args.size());

  if(req.command.compare(dviz_core::Command::Request::KILL_USER) == 0)
  {
    ok_ = false;
  } // end KILL_USER
  else if(req.command.compare(dviz_core::Command::Request::USER_INFO) == 0)
  {
    getUserProcessInfo();
  } // end USER_INFO
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
	// Load the task, and randomize it
	demonstration_scene_manager_->loadTask(ss.str(), true, 6);
      }
      else
      {
	// Load the task, and randomize it
	demonstration_scene_manager_->loadTask(req.args[0], true, 6);
      }
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for load_task (%d given, 1 required)",
		id_, (int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_task (" << req.args.size() << " given, 1 required)";
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

      res.response = "Done loading scene.";
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for load_scene (%d given, 1 required).",
		id_, (int)req.args.size());
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
		id_, (int)req.args.size());
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
      demonstration_scene_manager_->addMeshFromFile(req.args[0], 20, name, movable, false);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for load_mesh (%d given, 2 required, 1 optional).",
		id_, (int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_mesh (" << req.args.size() << " given, 2 required, 1 optional).";
      res.response = ss.str();
      return false;
    }    
  } // end LOAD_MESH
  else if(req.command.compare(dviz_core::Command::Request::GRIPPER_CONTROLS) == 0)
  {
    if(req.args.size() == 0)
    {
      // Enable advanced gripper and arm controls.
      simulator_->enableOrientationControl();
      simulator_->enableUpperArmRollControl();
    }
    else if(req.args.size() == 1)
    {
      bool basic = req.args[0].compare("true") == 0;
      if(basic)
      {
	// Disable advanced gripper and arm controls
	simulator_->disableOrientationControl();
	simulator_->disableUpperArmRollControl();
      }
      else
      {
	// Enable advanced gripper and arm controls
	simulator_->enableOrientationControl();
	simulator_->enableUpperArmRollControl();
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for gripper_controls (%d given, 1 optional).",
    		(int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for gripper_controls (" << req.args.size() << " given, 1 optional).";
      res.response = ss.str();
      return false;
    }    
  } // end GRIPPER_CONTROLS
  else if(req.command.compare(dviz_core::Command::Request::RESET_TASK) == 0)
  {
    resetTask();
  } // end RESET_TASK
  else if(req.command.compare(dviz_core::Command::Request::BEGIN_RECORDING) == 0)
  {
    if(req.args.size() == 1 || // Just user string provided
       req.args.size() == 2 || // User string and demo name provided
       req.args.size() == 3 || // User string, demo name, and bagfile name provided
       req.args.size() == 4)   // User string, demo name, bagfile name, and path to bagfile directory on the server provided
      
    {
      ROS_INFO("[DVizUser%d] Beginning to record demonstration of task %s",
	       id_, demonstration_scene_manager_->getTaskName().c_str());
      if(req.args.size() == 1)
      {
	// Just user string provided; use defaults for path and bagfile name
	recorder_->beginRecording(req.args[0], MotionRecorder::DEFAULT_DEMONSTRATION_PATH, "", demonstration_scene_manager_->getTaskName());
      }
      else if(req.args.size() == 2)
      {
	// User string and demo name provided; use default path
	recorder_->beginRecording(req.args[0], MotionRecorder::DEFAULT_DEMONSTRATION_PATH, req.args[1], demonstration_scene_manager_->getTaskName());
      }
      else if(req.args.size() == 3)
      {
	// User string, demo name, and bagfile name provided
	recorder_->beginRecording(req.args[0], MotionRecorder::DEFAULT_DEMONSTRATION_PATH, req.args[1], demonstration_scene_manager_->getTaskName(), req.args[2]);
      }
      else if(req.args.size() > 3)
      {
	// User string, demo name, bagfile name, and path provided
	recorder_->beginRecording(req.args[0], req.args[3], req.args[1], demonstration_scene_manager_->getTaskName(), req.args[2]);
      }

      if(demonstration_scene_manager_->getNumGoals() > 0)
      {
	int current_goal = demonstration_scene_manager_->getCurrentGoal();
	Goal *goal = demonstration_scene_manager_->getGoal(current_goal);

	std::string object_label = "";
	// Object type is only relevant if the the goal type is PICK_UP
	if(goal->getType() == Goal::PICK_UP)
	{
	  object_label = object_manager_->getObjectLabel(static_cast<PickUpGoal *>(goal)->getObjectID());
	}
	recorder_->addStep(Goal::GoalTypeNames[goal->getType()], object_label,
			   demonstration_scene_manager_->getGraspPose(current_goal));
      }
      else
      {
	ROS_WARN("[DVizUser%d] Recording user demonstration, but no task information loaded.", id_);
	recorder_->addStep("?", "?", geometry_msgs::Pose());
      }
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for begin_recording (%d given, 1 optional).",
		id_, (int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for begin_recording (" << req.args.size() << " given, 1 optional).";
      res.response = ss.str();
      return false;
    }
  } // end BEGIN_RECORDING
  else if(req.command.compare(dviz_core::Command::Request::NUM_FRAMES_IN_RECORDING) == 0)
  {
    int frames = std::min(recorder_->getNumPoses(), recorder_->getNumJoints());
    std::stringstream ss;
    ss << frames;
    res.response = ss.str();
  } // end NUM_FRAMES_IN_RECORDING
  else if(req.command.compare(dviz_core::Command::Request::BEGIN_REPLAY) == 0)
  {
    if(req.args.size() == 1)
    {
      recorder_->beginReplay(req.args[0]);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for begin_replay (%d given, 1 required).",
		id_, (int)req.args.size());
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
  else if(req.command.compare(dviz_core::Command::Request::FAST_FORWARD_REPLAY) == 0)
  {
    if(!recorder_->isReplaying())
    {
      ROS_ERROR("[DVizUser%d] Cannot fast-forward when not replaying", id_);
      std::stringstream ss;
      ss << "Cannot fast-forward when not replaying";
      res.response = ss.str();
      return false;
    }
    else
    {
      if(req.args.size() == 1)
      {
	int index = atoi(req.args[0].c_str());
	if(!recorder_->goTo(index))
	{
	  ROS_ERROR("[DVizUser%d] Invalid index to fast-forward to (%d)", id_, index);
	  std::stringstream ss;
	  ss << "Invalid index to fast-forward to (%d)" << index;
	  res.response = ss.str();
	  return false;
	}
      }
      else
      {
	ROS_ERROR("[DVizUser%d] Invalid number of arguments for fast_forward_replay (%d given, 1 required)",
		  id_, (int)req.args.size());
	std::stringstream ss;
	ss << "Invalid number of arguments for fast_forward_replay (" << req.args.size() << " given, 1 required)";
	res.response = ss.str();
	return false;
      }
    }
  } // end FAST_FORWARD_REPLAY
  else if(req.command.compare(dviz_core::Command::Request::SAVE_RECORDING) == 0)
  {
    recorder_->saveRecording();
  } // end SAVE_RECORDING
  else if(req.command.compare(dviz_core::Command::Request::PROCESS_KEY) == 0)
  {
    if(req.args.size() == 2)
    {
      int key = atoi(req.args[0].c_str());
      int type = atoi(req.args[1].c_str());

      processKeyEvent(key, type);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for process_key (%d given, 2 required).",
		id_, (int)req.args.size());
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
		id_, (int)req.args.size());
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
      //ROS_INFO("[DVizUser%d] Setting end-effector speed to %f", id_, speed);
      // Speed is given in m/s, must be converted to m/f.
      simulator_->setEndEffectorSpeed(speed * (1/getFrameRate()));
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for set_arm_speed (%d given, 1 required).",
		id_, (int)req.args.size());
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
		id_, (int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for set_frame_rate (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end SET_FRAME_RATE
  else if(req.command.compare(dviz_core::Command::Request::CHANGE_GOAL) == 0)
  {
    if(req.args.size() == 1)
    {
      int new_goal = atoi(req.args[0].c_str());
      ROS_INFO("[DVizUser%d] Changing goal number %d to the current goal.", id_, new_goal);
      demonstration_scene_manager_->setCurrentGoal(new_goal);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for change_goal (%d given, 1 required).",
		id_, (int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for change_goal (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end CHANGE_GOAL
  else if(req.command.compare(dviz_core::Command::Request::ACCEPT_GRASP) == 0)
  {
    int current_goal = demonstration_scene_manager_->getCurrentGoal();
    Goal *goal = demonstration_scene_manager_->getGoal(current_goal);
    if(goal->getType() == Goal::PICK_UP)
    {
      PickUpGoal *g = static_cast<PickUpGoal *>(goal);
      g->setGraspDone(true);
      demonstration_scene_manager_->setGoalsChanged();
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Goal must be of type PICK_UP to accept grasp!", id_);
      res.response = "Goal must be of type PICK_UP to accept grasp!";
      return false;
    }
  } // end ACCEPT_GRASP
  else if(req.command.compare(dviz_core::Command::Request::SHOW_INTERACTIVE_GRIPPER) == 0)
  {
    if(req.args.size() == 1)
    {
      int goal_number = atoi(req.args[0].c_str());

      if(!showInteractiveGripper(goal_number))
      {
	ROS_ERROR("[DVizUser%d] Cannot show interactive gripper for goal number %d.", id_, goal_number);
	return false;
      }
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for show_interactive_gripper (%d given, 1 required).",
		(int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for show_interactive_gripper (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end SHOW_INTERACTIVE_GRIPPER
  else if(req.command.compare(dviz_core::Command::Request::HIDE_INTERACTIVE_GRIPPER) == 0)
  {
    if(req.args.size() == 1)
    {
      int goal_number = atoi(req.args[0].c_str());

      if(!hideInteractiveGripper(goal_number))
      {
	ROS_ERROR("[DVizUser%d] Cannot hide interactive gripper for goal number %d.", id_, goal_number);
	return false;
      }
    } 
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for hide_interactive_gripper (%d given, 1 required).",
		(int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for hide_interactive_gripper (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end HIDE_INTERACTIVE_GRIPPER
  else if(req.command.compare(dviz_core::Command::Request::ROBOT_MARKER_CONTROL) == 0)
  {
    if(req.args.size() == 1)
    {
      bool enabled = (req.args[0].compare("true") == 0);
      
      // Enable or disable control of the robot via interactive markers
      simulator_->setMoveRobotMarkers(enabled);
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for robot_marker_control (%d given, 1 required).",
		(int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for robot_marker_control (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end ROBOT_MARKER_CONTROL
  else if(req.command.compare(dviz_core::Command::Request::SET_GRIPPER_JOINT) == 0)
  {
    if(req.args.size() == 1)
    {
      // Check that the current goal is of type PICK_UP; otherwise,
      // we cannot change the gripper joint angle
      Goal *current_goal = demonstration_scene_manager_->getGoal(
	demonstration_scene_manager_->getCurrentGoal());
      if(current_goal != 0 && current_goal->getType() == Goal::PICK_UP)
      {
	double gripper_joint = atof(req.args[0].c_str());

	PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(current_goal);
	pick_up_goal->setGripperJointPosition(gripper_joint);

	//showInteractiveGripper(demonstration_scene_manager_->getCurrentGoal());
	updateGripperMarkers(demonstration_scene_manager_->getCurrentGoal());
      }
      else
      {
	res.response = "Invalid goal type";
	return false;
      }
    }
    else
    {
      ROS_ERROR("[DVizUser%d] Invalid number of arguments for set_gripper_joint (%d given, 1 required).",
		(int)req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for set_gripper_joint (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end SET_GRIPPER_JOINT
  else if(req.command.compare(dviz_core::Command::Request::WRITE_STATS) == 0)
  {
    writeStats();
  } // end WRITE_STATS
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
      goal.gripper_joint_position = pick_up_goal->getGripperJointPosition();
      goal.camera_phi = pick_up_goal->getCameraPhi();
      goal.camera_theta = pick_up_goal->getCameraTheta();
      goal.camera_radius = pick_up_goal->getCameraRadius();

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
  {
    //ROS_INFO("[DVizUser%d] All goals complete; task complete!", id_);
    return;
  }

  Goal *current_goal = demonstration_scene_manager_->getGoal(demonstration_scene_manager_->getCurrentGoal());

  switch(current_goal->getType())
  {
  case Goal::PICK_UP:
  {
    PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(current_goal);
    bool goal_reachable = true;
      
    if(pick_up_goal->isComplete(simulator_->getEndEffectorPose()) && !simulator_->isBaseMoving()
       && accepted_grasp_)
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
      KDL::Frame gripper_in_map(KDL::Rotation::Quaternion(gripper_pose.orientation.x,
							  gripper_pose.orientation.y,
							  gripper_pose.orientation.z,
							  gripper_pose.orientation.w),
				KDL::Vector(gripper_pose.position.x,
					    gripper_pose.position.y,
					    gripper_pose.position.z)
	);

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

      // @todo buggy!
//      double dist_A = angles::normalize_angle_positive(roll) - angles::normalize_angle_positive(current_roll);
//      double dist_B = angles::normalize_angle_positive(roll + M_PI) - angles::normalize_angle_positive(current_roll);
//      if(std::abs(dist_A) > std::abs(dist_B))
//      {
//	KDL::Rotation rot = KDL::Rotation::RPY(roll + M_PI, pitch, yaw);
//	rot.GetQuaternion(x, y, z, w);
//	goal_gripper_pose.orientation.x = x;
//	goal_gripper_pose.orientation.y = y;
//	goal_gripper_pose.orientation.z = z;
//	goal_gripper_pose.orientation.w = w;

//	gripper_in_map.M.GetRPY(roll, pitch, yaw);
//	gripper_in_map.M = KDL::Rotation::RPY(roll + M_PI, pitch, yaw);
//	// marker_in_map.M.GetRPY(roll, pitch, yaw);
//	// marker_in_map.M = KDL::Rotation::RPY(roll + M_PI, pitch, yaw);
//	//gripper_in_map = marker_in_map * gripper_in_marker;
//	object_in_gripper = gripper_in_map.Inverse() * object_in_map;
//      }

      // For a pick up goal, allow the gripper to be in collision with
      // the object that it must pick up (our collision spheres may not
      // be precise enough to allow for tight grasps of the object while
      // still being free of collision)
      goal_reachable = simulator_->snapEndEffectorTo(goal_gripper_pose,
						     pick_up_goal->getGripperJointPosition(),
						     false,
	                                             true,
                                                     true,
	                                             true,
	                                             true,
	                                             pick_up_goal->getObjectID());

      if(goal_reachable)
      {
	simulator_->attach(pick_up_goal->getObjectID(), object_in_gripper);
	  
	demonstration_scene_manager_->setCurrentGoal(demonstration_scene_manager_->getCurrentGoal() + 1);

	goalCompleted();
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

    if(place_goal->isComplete(object_pose) && !simulator_->isBaseMoving())
    {
      // Snap the gripper to the correct position so that the object that it is holding moves 
      // smoothly to the goal pose. 
      geometry_msgs::Pose object_goal_pose = place_goal->getPlacePose();
      if(place_goal->ignoreYaw())
      {
	geometry_msgs::Pose object_pose;
	if(!simulator_->getObjectPose(object_pose))
	{
	  ROS_ERROR("[DVizUser%d] No attached object!", id_);
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
	  
	demonstration_scene_manager_->setCurrentGoal(demonstration_scene_manager_->getCurrentGoal() + 1);

	goalCompleted();
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
  task_pub_ = nh.advertise<dviz_core::Task>("dviz_task", 500);

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

void DemonstrationVisualizerUser::getUserProcessInfo()
{
  double ram = ProcessInfo::getProcessPhysicalMemory();
  ROS_INFO("[DVizUser%d] Physical memory in use: %0.4f", id_, ram);
  double vmem = ProcessInfo::getProcessVirtualMemory();
  ROS_INFO("[DVizUser%d] Virtual memory in use: %0.4f", id_, vmem);
  double cpu = ProcessInfo::getProcessCPU();
  if (cpu < 0)
    ROS_ERROR("[DVizUser%d] Invalid CPU percent!", id_);
  else
    ROS_INFO("[DVizUser%d] Percent CPU in use: %0.4f", id_, cpu);
}

void DemonstrationVisualizerUser::writeStats()
{
  ROS_INFO("[DVizUser%d] Writing stats...", id_);
  std::stringstream ss;
  ss << ros::Time::now().sec << " " << "USER" << " " << id_ << " " << ProcessInfo::getProcessCPU() << " "
     << ProcessInfo::getProcessPhysicalMemory() << " " << ProcessInfo::getProcessVirtualMemory() << " "
     << ProcessInfo::getTotalCPU() << " " << ProcessInfo::getTotalPhysicalMemory() << " "
     << ProcessInfo::getTotalVirtualMemory() << "\n";
  ProcessInfo::writeStats(ss.str(), true);
}

void DemonstrationVisualizerUser::goalCompleted()
{
  ROS_INFO("[DVizUser%d] Goal completed!", id_);
  //writeStats();

  if(recorder_->isRecording())
  {
    if(demonstration_scene_manager_->taskDone())
    {
      ROS_INFO("[DVizUser%d] Task done!", id_);
      // Add an end-of-task (EOT) step
      recorder_->addStep("", "EOT", geometry_msgs::Pose());
    }
    else if(demonstration_scene_manager_->getNumGoals() > 0)
    {
      int current_goal = demonstration_scene_manager_->getCurrentGoal();
      Goal *goal = demonstration_scene_manager_->getGoal(current_goal);
      if (!goal)
      {
        ROS_ERROR("[DVizUser%d] In goal completed, no such goal number %d!", id_, current_goal);
        return;
      }
      std::string object_label = "";
      geometry_msgs::Pose grasp_pose;
      // Object type and grasp pose is only relevant if the the goal type is PICK_UP
      if(goal->getType() == Goal::PICK_UP)
      {
	object_label = object_manager_->getObjectLabel(static_cast<PickUpGoal *>(goal)->getObjectID());
	grasp_pose = demonstration_scene_manager_->getGraspPose(current_goal);
      }
      else if(goal->getType() == Goal::PLACE)
      {
	object_label = object_manager_->getObjectLabel(static_cast<PlaceGoal *>(goal)->getObjectID());
      }
      ROS_INFO("[DVizUser%d] Adding step for goal type %s and object %s", id_,
	       Goal::GoalTypeNames[goal->getType()], object_label.c_str());
      recorder_->addStep(Goal::GoalTypeNames[goal->getType()], object_label, grasp_pose);
    }
    else
    {
      ROS_WARN("[DVizUser%d] No task loaded, so recording with no user demonstration information", id_);
      recorder_->addStep("?", "?", geometry_msgs::Pose());
    }
  }
}

bool DemonstrationVisualizerUser::showInteractiveGripper(int goal_number)
{
  accepted_grasp_ = false;

  if(!demonstration_scene_manager_->getGoal(goal_number) || demonstration_scene_manager_->getGoal(goal_number)->getType() != Goal::PICK_UP)
  {
    ROS_ERROR("[DVizUser%d] Cannot show a gripper for a goal that is not of type PICK_UP", id_);
    return false;
  }

  PickUpGoal *goal = static_cast<PickUpGoal *>(demonstration_scene_manager_->getGoal(goal_number));

  geometry_msgs::Pose gripper_pose = goal->getGraspPose();

  // // Find the pose of the gripper in the map
  // KDL::Frame center_in_map(KDL::Rotation::Quaternion(
  // 			     gripper_pose.orientation.x,
  // 			     gripper_pose.orientation.y,
  // 			     gripper_pose.orientation.z,
  // 			     gripper_pose.orientation.w),
  // 			   KDL::Vector(
  // 			     gripper_pose.position.x,
  // 			     gripper_pose.position.y,
  // 			     gripper_pose.position.z)
  //   );
  // KDL::Frame gripper_in_center(KDL::Rotation::Identity(),
  // 			       KDL::Vector(
  // 				 /*-(goal->getGraspDistance())*/0,
  // 				 0,
  // 				 0
  // 				 )
  //   );
  // KDL::Frame gripper_in_map = center_in_map * gripper_in_center;
  // geometry_msgs::Pose gripper_pose_map;
  // gripper_pose_map.position.x = gripper_in_map.p.x();
  // gripper_pose_map.position.y = gripper_in_map.p.y();
  // gripper_pose_map.position.z = gripper_in_map.p.z();
  // double x, y, z, w;
  // gripper_in_map.M.GetQuaternion(x, y, z, w);
  // gripper_pose_map.orientation.x = x;
  // gripper_pose_map.orientation.y = y;
  // gripper_pose_map.orientation.z = z;
  // gripper_pose_map.orientation.w = w;

  // // Visualize the gripper using a marker array
  visualization_msgs::MarkerArray gripper_markers;
  std::stringstream s;
  s << "grasp_marker_goal_" << goal_number;
  pviz_->getGripperMeshesMarkerMsg(gripper_pose, 0.2, s.str(), 1, goal->getGripperJointPosition(), gripper_markers.markers);
  // std::vector<visualization_msgs::Marker>::iterator it;
  // for(it = gripper_markers.markers.begin(); 
  //     it != gripper_markers.markers.end();
  //     ++it)
  // {
  //   it->header.frame_id = resolveName("/map", id_);
  // }
  // marker_array_pub_.publish(gripper_markers);
  updateGripperMarkers(goal_number);

  // Use a separate interactive marker to control the pose of the 
  // grasp, and for opening and closing the gripper
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = resolveName("/map", id_);
  int_marker.name = s.str();
  int_marker.description = "";
  int_marker.pose = gripper_pose;
  int_marker.scale = 0.5;

  visualization_msgs::InteractiveMarkerControl control;
  visualization_msgs::Marker gripper_wrist_marker;
  gripper_wrist_marker = gripper_markers.markers.at(0);
  gripper_wrist_marker.header.frame_id = "";
  gripper_wrist_marker.pose.position.x = 0;
  gripper_wrist_marker.pose.position.y = 0;
  gripper_wrist_marker.pose.position.z = 0;
  gripper_wrist_marker.pose.orientation.x = 0;
  gripper_wrist_marker.pose.orientation.y = 0;
  gripper_wrist_marker.pose.orientation.z = 0;
  gripper_wrist_marker.pose.orientation.w = 1;
  control.markers.push_back(gripper_wrist_marker);

  // Users are able to move the gripper marker in the plane
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
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

  control.name = "OPEN_GRIPPER_ARROW";
  visualization_msgs::Marker arrow;
  arrow.header.frame_id = "";
  arrow.pose.position.y = 0;
  arrow.pose.position.x = -0.06;
  arrow.pose.position.z = 0;
  arrow.pose.orientation.z = 1;
  arrow.pose.orientation.w = 1;
  arrow.type = visualization_msgs::Marker::ARROW;
  arrow.color.r = 1.0;
  arrow.color.g = 0.0;
  arrow.color.b = 0.0;
  arrow.color.a = 1.0;
  arrow.scale.x = 0.06;
  arrow.scale.y = 0.04;
  control.markers.push_back(arrow);
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::BUTTON;

  arrow.pose.position.x = -0.06;
  arrow.pose.orientation.z = -1;
  control.markers.push_back(arrow);
  int_marker.controls.push_back(control);

  control.markers.clear();
  control.name = "CLOSE_GRIPPER_ARROW";
  arrow.pose.position.x = -0.08;
  arrow.pose.position.y = 0.06;
  arrow.pose.orientation.z = -1;
  control.markers.push_back(arrow);

  arrow.pose.position.x = -0.08;
  arrow.pose.position.y = -0.06;
  arrow.pose.orientation.z = 1;
  control.markers.push_back(arrow);
  int_marker.controls.push_back(control);

  int_marker_server_->insert(int_marker,
  			     boost::bind(
  			       &DemonstrationVisualizerUser::gripperMarkerFeedback,
  			       this,
  			       _1)
  			     );
  int_marker_server_->applyChanges();

  return true;
}

bool DemonstrationVisualizerUser::hideInteractiveGripper(int goal_number)
{
  accepted_grasp_ = true;

  if(demonstration_scene_manager_->getGoal(goal_number)->getType() != Goal::PICK_UP)
  {
    ROS_ERROR("[DVizUser%d] Cannot hide a gripper for a goal that is not of type PICK_UP.", id_);
    return false;
  }

  // Get pose of gripper in object frame
  // geometry_msgs::Pose gripper_pose_in_object = 
  //   demonstration_scene_manager_->getGraspPoseObjectFrame(goal_number);

  visualization_msgs::MarkerArray gripper_markers;
  std::stringstream s;
  s << "grasp_marker_goal_" << goal_number;
  geometry_msgs::Pose origin;
  origin.orientation.w = 1;
  pviz_->getGripperMeshesMarkerMsg(origin, 0.2, s.str(), 1, 0.0, gripper_markers.markers);
  std::vector<visualization_msgs::Marker>::iterator it;
  for(it = gripper_markers.markers.begin(); 
      it != gripper_markers.markers.end();
      ++it)
  {
    it->header.frame_id = resolveName("/map", id_);
    it->action = visualization_msgs::Marker::DELETE;
  }
  marker_array_pub_.publish(gripper_markers);

  s.str(std::string());
  s << "grasp_marker_goal_" << goal_number;

  int_marker_server_->erase(s.str());
  int_marker_server_->applyChanges();

  return true;
}

void DemonstrationVisualizerUser::gripperMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  int i = feedback->marker_name.size()-1;
  for(; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  int goal_number = atoi(feedback->marker_name.substr(i+1).c_str());

  Goal *goal = demonstration_scene_manager_->getGoal(goal_number);
  if(!goal)
  {
    ROS_ERROR("[DVizUser%d] In gripper marker feedback, no such goal %d!", id_, goal_number);
    return;
  }
  if(goal->getType() == Goal::PICK_UP)
  {
    // geometry_msgs::Pose gripper_pose = static_cast<PickUpGoal *>(goal)->getGraspPose();

    // KDL::Frame grasp_in_map(KDL::Rotation::Quaternion(
    // 			      gripper_pose.orientation.x,
    // 			      gripper_pose.orientation.y,
    // 			      gripper_pose.orientation.z,
    // 			      gripper_pose.orientation.w),
    // 			    KDL::Vector(gripper_pose.position.x,
    // 					gripper_pose.position.y,
    // 					gripper_pose.position.z)
    //   );

    // KDL::Frame arrow_in_grasp(KDL::Rotation::Quaternion(0, 0, 1, 1),
    // 			      KDL::Vector(-(static_cast<PickUpGoal *>(goal)->getGraspDistance()), 0, 0));

    // KDL::Frame arrow_in_map = grasp_in_map * arrow_in_grasp;
    // double x, y, z, w;
    // arrow_in_map.M.GetQuaternion(x, y, z, w);
    // geometry_msgs::Pose grasp_arrow_pose;
    // grasp_arrow_pose.position.x = arrow_in_map.p.x();
    // grasp_arrow_pose.position.y = arrow_in_map.p.y();
    // grasp_arrow_pose.position.z = arrow_in_map.p.z();
    // grasp_arrow_pose.orientation.x = x;
    // grasp_arrow_pose.orientation.y = y;
    // grasp_arrow_pose.orientation.z = z;
    // grasp_arrow_pose.orientation.w = w;

    // std::stringstream ss;
    // ss << "grasp_angle_marker_goal_" << goal_number;
    
    // int_marker_server_->setPose(ss.str(), grasp_arrow_pose);

    if(feedback->control_name.compare("OPEN_GRIPPER_ARROW") == 0 ||
       feedback->control_name.compare("CLOSE_GRIPPER_ARROW") == 0)
    {
      bool open = feedback->control_name.compare("OPEN_GRIPPER_ARROW") == 0;
      //ROS_INFO_STREAM((open ? "open " : "close ") << "arrow clicked");
      Goal *current_goal = demonstration_scene_manager_->getGoal(
	demonstration_scene_manager_->getCurrentGoal());
      if(current_goal != 0 && current_goal->getType() == Goal::PICK_UP)
      {
	PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(current_goal);

	double increment = 0.02;
	if(!open) increment *= -1.0;

	// Check to make sure that the new gripper angle is within bounds
	double new_angle = pick_up_goal->getGripperJointPosition() + increment;
	if(new_angle < EndEffectorController::GRIPPER_CLOSED_ANGLE)
	  new_angle = EndEffectorController::GRIPPER_CLOSED_ANGLE;
	else if(new_angle > EndEffectorController::GRIPPER_OPEN_ANGLE)
	  new_angle = EndEffectorController::GRIPPER_OPEN_ANGLE;

	pick_up_goal->setGripperJointPosition(new_angle);

	updateGripperMarkers(demonstration_scene_manager_->getCurrentGoal());
	// showInteractiveGripper(demonstration_scene_manager_->getCurrentGoal());
      }
    }
    else
    {
      // Adjust the orientation of the controls, so that the axes remain at a fixed orientation
      if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
      {
	visualization_msgs::InteractiveMarker gripper_marker;
	int_marker_server_->get(feedback->marker_name, gripper_marker);
	tf::Quaternion rot1(feedback->pose.orientation.x,
			    feedback->pose.orientation.y,
			    feedback->pose.orientation.z,
			    feedback->pose.orientation.w);
	tf::Quaternion rot2(tf::Vector3(0, 1.0, 0), M_PI/2.0);
	tf::quaternionTFToMsg(rot1.inverse() * rot2, gripper_marker.controls.at(0).orientation);

	int_marker_server_->insert(gripper_marker);
	int_marker_server_->applyChanges();

	updateGripperMarkers(demonstration_scene_manager_->getCurrentGoal());
//	showInteractiveGripper(demonstration_scene_manager_->getCurrentGoal());
      }

      PickUpGoal *p = static_cast<PickUpGoal *>(goal);
      // Check if the gripper is too far from the object
      geometry_msgs::Pose object_pose = p->getInitialObjectPose();
      double dx = object_pose.position.x - feedback->pose.position.x;
      double dy = object_pose.position.y - feedback->pose.position.y;
      double dz = object_pose.position.z - feedback->pose.position.z;
      double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

      // Adjust the control orientation of the gripper appropriately
      // visualization_msgs::InteractiveMarker gripper_marker;
      // int_marker_server_->get(feedback->marker_name, gripper_marker);
      // tf::Quaternion rot(feedback->pose.orientation.x,
      // 		       feedback->pose.orientation.y,
      // 		       feedback->pose.orientation.z,
      // 		       feedback->pose.orientation.w);
      // tf::Quaternion rot2(tf::Vector3(0, 1, 0), M_PI/2.0);
      // tf::quaternionTFToMsg(rot.inverse() * rot2, gripper_marker.controls.at(0).orientation);

      // int_marker_server_->insert(gripper_marker);
      // int_marker_server_->applyChanges();

      // If the gripper marker has been moved too far from the 
      // position of the object, do not do anything
      if(distance > 0.3)
      {
	ROS_WARN("[DVizUser%d] The grasp is too far from object", id_);
	// Reset the pose of the interactive gripper
	if(feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP)
	{
	  geometry_msgs::Pose last_pose = demonstration_scene_manager_->getGraspPose(goal_number);

	  int_marker_server_->setPose(feedback->marker_name, last_pose);
	  int_marker_server_->applyChanges();
	}
	return;
      }
    }
  }

  demonstration_scene_manager_->setGraspPose(goal_number, feedback->pose);
}

// void DemonstrationVisualizerUser::graspArrowMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
// {
//   // @todo
// }


void DemonstrationVisualizerUser::processKeyEvent(int key, int type)
{
  //ROS_INFO("[DVizUser%d] Processing key %d, event type %d.", id_, key, type);

  switch(type)
  {
  case QEvent::KeyPress:
  {
    switch(key)
    {
    case Qt::Key_Z:
    {
      // Allow the user to move any interacive gripper along the z-axis
      std::vector<Goal *> goals = demonstration_scene_manager_->getGoals();
      std::vector<Goal *>::const_iterator it = goals.begin();
      for(; it != goals.end(); ++it)
      {
	std::stringstream ss;
	// All interactive gripper markers have a name which is 
	// uniquely identified with the goal number
	ss << "grasp_marker_goal_" << (*it)->getGoalNumber();
	visualization_msgs::InteractiveMarker gripper_marker;
	if(int_marker_server_->get(ss.str(), gripper_marker))
	{
	  gripper_marker.controls.at(0).interaction_mode = 
	    visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

	  int_marker_server_->insert(gripper_marker);
	  int_marker_server_->applyChanges();
	}
      }
      break;
    }
    default:
      break;
    }
    break;
  }
  case QEvent::KeyRelease:
  {
    switch(key)
    {
    case Qt::Key_Z:
    {
      std::vector<Goal *> goals = demonstration_scene_manager_->getGoals();
      std::vector<Goal *>::const_iterator it = goals.begin();
      for(; it != goals.end(); ++it)
      {
	std::stringstream ss;
	ss << "grasp_marker_goal_" << (*it)->getGoalNumber();
	visualization_msgs::InteractiveMarker gripper_marker;
	if(int_marker_server_->get(ss.str(), gripper_marker))
	{
	  gripper_marker.controls.at(0).interaction_mode = 
	    visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

	  int_marker_server_->insert(gripper_marker);
	  int_marker_server_->applyChanges();
	}
      }
      break;
    }
    default:
      break;
    }
    break;
  }
  default:
    ROS_ERROR("[DVizUser%d] Unrecognized key event type: %d", id_, type);
    return;
  }

  // Pass key events to the simulator
  simulator_->processKeyEvent(key, type);
}

// @todo move this to demonstration scene manager
void DemonstrationVisualizerUser::resetTask()
{
  ROS_INFO("[DVizUser%d] Resetting task", id_);
  if(demonstration_scene_manager_->getNumGoals() > 0)
  {
    // Reset to the first goal
    demonstration_scene_manager_->setCurrentGoal(0);

    // If the robot is holding an object, detach it
    if(simulator_->isObjectAttached())
      simulator_->detach();

    // Reset all the objects to their initial positions
    std::vector<Goal *> goals = demonstration_scene_manager_->getGoals();
    std::vector<Goal *>::const_iterator it;
    geometry_msgs::Pose default_pose;
    default_pose.position.x = default_pose.position.y = default_pose.position. z = 0;
    default_pose.orientation.x = default_pose.orientation. y = default_pose.orientation.z = 0;
    default_pose.orientation.w = 1;
    for(it = goals.begin(); it != goals.end(); ++it)
    {
      if((*it)->getType() == Goal::PICK_UP)
      {
	PickUpGoal *g = static_cast<PickUpGoal *>(*it);
	geometry_msgs::Pose initial_pose = g->getInitialObjectPose();
	int object_id = g->getObjectID();
	object_manager_->moveObject(object_id, initial_pose);
	g->setGraspPose(initial_pose);
	//g->setGraspDistance(0.25);
	g->setGraspDone(false);
	g->setGripperJointPosition(EndEffectorController::GRIPPER_OPEN_ANGLE);
      }
    }
  }
}

void DemonstrationVisualizerUser::pingResponse(const std_msgs::Empty &msg)
{
  ROS_INFO("[DVizUser%d] User's client is alive, recieved a ping", id_);
  ping_count_ = 0;
  ping_delay_count_ = 0;
}

void DemonstrationVisualizerUser::updateGripperMarkers(int goal_number)
{
  if(demonstration_scene_manager_->getGoal(goal_number)->getType() == Goal::PICK_UP)
  {
    PickUpGoal *goal = static_cast<PickUpGoal *>(demonstration_scene_manager_->getGoal(goal_number));

    geometry_msgs::Pose gripper_pose = goal->getGraspPose();

    // Visualize the gripper using a marker array
    visualization_msgs::MarkerArray gripper_markers;
    std::stringstream s;
    s << "grasp_marker_goal_" << goal_number;
    pviz_->getGripperMeshesMarkerMsg(gripper_pose, 0.2, s.str(), 1, goal->getGripperJointPosition(), gripper_markers.markers);
    std::vector<visualization_msgs::Marker>::iterator it;
    for(it = gripper_markers.markers.begin(); 
	it != gripper_markers.markers.end();
	++it)
    {
      it->header.frame_id = resolveName("/map", id_);
    }
    marker_array_pub_.publish(gripper_markers);
  }
}

} // namespace demonstration_visualizer
