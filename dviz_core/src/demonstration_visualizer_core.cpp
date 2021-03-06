#include <dviz_core/demonstration_visualizer_core.h>

namespace demonstration_visualizer
{

DemonstrationVisualizerCore::DemonstrationVisualizerCore(int argc, char **argv)
  : last_id_(1), num_users_(0), object_manager_(0), demonstration_scene_manager_(0), scene_loaded_(false), stats_counter_(0)
{
  if(!init(argc, argv))
    ROS_ERROR("[DVizCore] Unable to connect to master!");

  std::string rarm_filename;
  std::string larm_filename;
  ros::NodeHandle nh("~");
  std::string default_larm_filename = ros::package::getPath("pr2_collision_checker") + "/config/pr2_left_arm.cfg";
  std::string default_rarm_filename = ros::package::getPath("pr2_collision_checker") + "/config/pr2_right_arm.cfg";
  nh.param<std::string>("left_arm_description_file", larm_filename, default_larm_filename);
  nh.param<std::string>("right_arm_description_file", rarm_filename, default_rarm_filename);
  object_manager_ = new ObjectManager(rarm_filename, larm_filename, 0, true);
  demonstration_scene_manager_ = new DemonstrationSceneManager(0, 0, object_manager_, 0);
  ProcessInfo::initCPU();
}

DemonstrationVisualizerCore::~DemonstrationVisualizerCore()
{
  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }

  delete object_manager_;
  delete demonstration_scene_manager_;
}

bool DemonstrationVisualizerCore::init(int argc, char **argv)
{
  ros::init(argc, argv, "dviz_core");
  sleep(2.0);

  // Make sure that we can communicate with the master.
  if(!ros::master::check())
    return false;

  ros::start();

  ros::NodeHandle nh;

  // Advertise a service for processing commands issued to dviz.
  command_service_ = nh.advertiseService<dviz_core::Command::Request,
					 dviz_core::Command::Response>("/dviz_command",
					 boost::bind(static_cast<bool (DemonstrationVisualizerCore::*)
						                 (dviz_core::Command::Request &,
								  dviz_core::Command::Response &)>(
								    &DemonstrationVisualizerCore::processCommand),
						     this, _1, _2));

  return true;
}

bool DemonstrationVisualizerCore::processCommand(dviz_core::Command::Request &req,
						 dviz_core::Command::Response &res)
{
  // First, determine what the command is, then process the appropriate number of arguments
  if(req.command.compare(dviz_core::Command::Request::ADD_USER) == 0)
  {
    int id = addUser();
    if(id > 0)
    {
      std::stringstream ss;
      ss << id;
      res.response = ss.str();
    }
    else
    {
      ROS_ERROR("[DVizCore] Failed to add user!");
      res.response = "-1";
      return false;
    }
  } // end ADD_USER
  else if(req.command.compare(dviz_core::Command::Request::KILL_USER) == 0)
  {
    if(req.args.size() == 1)
    {
      int user_id = atoi(req.args[0].c_str());
      ROS_INFO("[DVizCore] Killing user %d", user_id);
      if(!passCommandToUser(dviz_core::Command::Request::KILL_USER, res.response, user_id))
      {
	ROS_ERROR("[DVizCore] Error in kill_user command");
      }
      else
      {
	//num_users_--;
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for kill_user (%d given, 1 required).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for kill_user (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end KILL_USER
  else if(req.command.compare(dviz_core::Command::Request::NUM_USERS) == 0)
  {
    std::stringstream ss;
    ss << num_users_;
    res.response = ss.str();
    return true;
  } // end NUM_USERS
  else if(req.command.compare(dviz_core::Command::Request::DECR_USERS) == 0)
  {
    num_users_--;
  }
  else if(req.command.compare(dviz_core::Command::Request::USER_INFO) == 0)
  {
    if(req.args.size() == 1)
    {
      int user_id = atoi(req.args[0].c_str());
      if(!passCommandToUser(dviz_core::Command::Request::USER_INFO, res.response, user_id))
      {
	ROS_ERROR("[DVizCore] Error is user_info (id = %d).", user_id);
      }
      // @todo send response string with info
    }
    else if(req.args.size() == 0)
    {
      // If no DVizUser id is provided, just list stats on all users.
      ROS_INFO("[DVizCore] Listing stats on %d users.", user_command_services_.size());
      std::map<int, ros::ServiceClient>::iterator it;
      for(it = user_command_services_.begin(); it != user_command_services_.end(); ++it)
      {
	if(!passCommandToUser(dviz_core::Command::Request::USER_INFO, res.response, it->first))
	{
	  ROS_ERROR("[DVizCore] Error is user_info (id = %d).", it->first);
	}
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for user_info (%d given, 1 required).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for user_info (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  } // end USER_INFO
  else if(req.command.compare(dviz_core::Command::Request::WRITE_STATS) == 0)
  {
    if(req.args.size() == 1)
    {
      int user_id = atoi(req.args[0].c_str());
      if(!passCommandToUser(dviz_core::Command::Request::WRITE_STATS, res.response, user_id))
      {
        ROS_ERROR("Failed to write stats for user %d", user_id);
      }
    }
    else
    {
      writeStats();
    }
  }
  else if(req.command.compare(dviz_core::Command::Request::LOAD_SCENE) == 0)
  {
    if(req.args.size() == 2)
    {
      int user_id = atoi(req.args[0].c_str());
      std::vector<std::string> args;
      args.push_back(req.args[1]);

      if(scene_loaded_)
	ROS_WARN("[DVizCore] Scene already loaded, skipping this step");

      // First load the appropriate scene into shared memory.
      if(!scene_loaded_ && demonstration_scene_manager_->loadScene(req.args[1]) > -1)
      {
	object_manager_->initSharedDistanceField();
	scene_loaded_ = true;
      }

      boost::thread worker(
	&DemonstrationVisualizerCore::passCommandToUserThreaded,
	this,
	dviz_core::Command::Request::LOAD_SCENE,
	res.response,
	user_id,
	args,
	10.0f);
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for load_scene (%d given, 2 required)",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_scene (" << req.args.size() << " given, 2 required)";
      res.response = ss.str();
      return false;
    }
  } // end LOAD_SCENE
  else if(req.command.compare(dviz_core::Command::Request::PASS_USER_COMMAND_THREADED) == 0)
  {
    if(req.args.size() >= 2)
    {
      int user_id = atoi(req.args[0].c_str());
      std::string command = req.args[1];
      std::vector<std::string> args;
      for(int i = 2; i < req.args.size(); ++i)
      {
	args.push_back(req.args[i]);
      }

      boost::thread worker(
	&DemonstrationVisualizerCore::passCommandToUserThreaded,
	this,
	command,
	res.response,
	user_id,
	args,
	10.0f);
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for pass_user_command_threaded (%d given, >= 2 required)",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for pass_user_command_threaded (" << req.args.size() << " given, >= 2 required)";
      res.response = ss.str();
      return false;
    }
  } // end PASS_USER_COMMAND_THREADED
  else
  {
    ROS_ERROR("[DVizCore] Invalid command \"%s\"", req.command.c_str());
    std::stringstream ss;
    ss << "Invalid command \"" << req.command.c_str() << "\"";
    res.response = ss.str();
    return false;
  }

  return true;
}

bool DemonstrationVisualizerCore::processCommand(const std::string &command, 
						 const std::vector<std::string> &args)
{
  dviz_core::Command::Request req;
  req.command = command;
  req.args = args;
  dviz_core::Command::Response res;
  return processCommand(req, res);
}

int DemonstrationVisualizerCore::addUser()
{
  int id = last_id_;
  last_id_ += 1;
  int rosrun = fork();
  if(rosrun == 0)
  {
    char user_id[256];
    sprintf(user_id, "%d", id);
    ROS_INFO("Child pid %d adding DVizUser with id %d.", getpid(), id);
    id = last_id_;
    execlp("rosrun", "rosrun", "dviz_core", "dviz_user_node", user_id, (char *)0);
    ROS_WARN("execlp probably failed in addUser()!");
    return -1;
  }
  else if(rosrun < 0)
  {
    ROS_ERROR("[DVizCore] fork failed in addUser()!");
    return -1;
  }

  num_users_++;
  ROS_INFO("[DVizCore] Number of users is %d", num_users_);

  // Establish a command service client to issue commands to the newly created user
  ros::NodeHandle nh;
  ROS_INFO("[DVizCore] Adding %s", resolveName("dviz_command", id).c_str());
  if(user_command_services_.find(id) == user_command_services_.end())
  {
    ROS_INFO("[DVizCore] Adding service %s", resolveName("dviz_command", id).c_str());
    user_command_services_.insert(std::pair<int, ros::ServiceClient>(
				    id, nh.serviceClient<dviz_core::Command>(resolveName("dviz_command", id))));
  }
  else
  {
    ROS_INFO("[DVizCore] Re-adding service %s", resolveName("dviz_command", id).c_str());
    user_command_services_[id] = nh.serviceClient<dviz_core::Command>(resolveName("dviz_command", id));
  }

  //writeStats();
  return id;
}

bool DemonstrationVisualizerCore::passCommandToUser(const std::string &command, 
						    std::string &response, 
						    int id,
                                                    const std::vector<std::string> &args)
{
  if(user_command_services_.find(id) != user_command_services_.end())
  {
    dviz_core::Command srv;
    srv.request.command = command;
    if(args.size() > 0)
    {
      srv.request.args = args;
    }

    if(user_command_services_[id].call(srv))
    {
      //ROS_INFO("[DVizCore] Passed %s command to user ID %d.", command.c_str(), id);
    }
    else
    {
      ROS_ERROR("[DVizCore] Something went wrong in passing %s command to user ID %d!", command.c_str(), id);
      std::stringstream ss;
      ss << "Something went wrong in passing " << command << " command to user ID " << id;
      response = ss.str();
      return false;
    }
  }
  else
  {
    ROS_ERROR("[DVizCore] User with ID %d not found!", id);
    std::stringstream ss;
    ss << "In command " << command << ", user with ID " << id << " not found!";
    response = ss.str();
    return false;
  }

  return true;
}

void DemonstrationVisualizerCore::passCommandToUserThreaded(
  const std::string &command,
  std::string &response,
  int id,
  const std::vector<std::string> &args,
  float rate)
{
  // First, check if the user exists
  if(user_command_services_.find(id) == user_command_services_.end())
  {
    ROS_ERROR("[DVizCore] In threaded pass command, user with id %d does not exist", id);
    return;
  }

  bool success = false;

  ros::Rate r(rate);

  // Try until success
  while(!success)
  {
    success = passCommandToUser(command, response, id, args);

    if(rate == 0) break;

    r.sleep();
  }
}

void DemonstrationVisualizerCore::run()
{
  ros::Rate rate(10.0);
  while(ros::ok())
  {
    // if(stats_counter_ % 200 == 0)
    //   writeStats();

    // stats_counter_++;
    ros::spinOnce();
    rate.sleep();
  }
}

void DemonstrationVisualizerCore::writeStats()
{
  ROS_INFO("[DVizCore] Writing stats...");
  // Write the system stats
  std::stringstream ss;
  ss << ros::Time::now().sec << " " << "CORE" << " " << num_users_ << " " << ProcessInfo::getTotalCPU() << " "
     << ProcessInfo::getUsedPhysicalMemory() << " " << ProcessInfo::getUsedVirtualMemory() << "\n";
  ProcessInfo::writeStats(ss.str(), true);
  // // Write the process stats for each user process
  // for(std::map<int, ros::ServiceClient>::iterator it = user_command_services_.begin();
  //     it != user_command_services_.end(); ++it)
  // {
  //   std::string response;
  //   if(!passCommandToUser(dviz_core::Command::Request::WRITE_STATS, response, it->first))
  //   {
  //     ROS_WARN("Failed to write stats for user %d", it->first);
  //   }
  // }
}

} // namespace demonstration_visualizer
