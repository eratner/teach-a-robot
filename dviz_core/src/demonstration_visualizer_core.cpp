#include <dviz_core/demonstration_visualizer_core.h>

namespace demonstration_visualizer
{

DemonstrationVisualizerCore::DemonstrationVisualizerCore(int argc, char **argv)
  : last_id_(1)
{
  if(!init(argc, argv))
    ROS_ERROR("[DVizCore] Unable to connect to master!");
}

DemonstrationVisualizerCore::~DemonstrationVisualizerCore()
{
  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
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
  // First, determine what the command is. Then process the appropriate number of arguments.
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
  }
  else if(req.command.compare(dviz_core::Command::Request::KILL_USER) == 0)
  {
    if(req.args.size() == 1)
    {
      int user_id = atoi(req.args[0].c_str());
      if(!passCommandToUser(dviz_core::Command::Request::KILL_USER, res.response, user_id))
      {
	ROS_ERROR("[DVizCore] Error in kill_user command.");
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
  }
  else if(req.command.compare(dviz_core::Command::Request::PLAY) == 0)
  {
    if(req.args.size() == 1)
    {
      int user_id = atoi(req.args[0].c_str());
      if(!passCommandToUser(dviz_core::Command::Request::PLAY, res.response, user_id))
      {
	ROS_ERROR("[DVizCore] Error in play command.");
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for play (%d given, 1 required).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for play (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  }
  else if(req.command.compare(dviz_core::Command::Request::PAUSE_NOW) == 0)
  {
    if(req.args.size() == 1)
    {
      int user_id = atoi(req.args[0].c_str());
      if(!passCommandToUser(dviz_core::Command::Request::PAUSE_NOW, res.response, user_id))
      {
	ROS_ERROR("[DVizCore] Error in pause_now command.");
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for pause_now (%d given, 1 required).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for pause_now (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  }
  else if(req.command.compare(dviz_core::Command::Request::PAUSE_LATER) == 0)
  {
    if(req.args.size() == 1)
    {
      int user_id = atoi(req.args[0].c_str());
      if(!passCommandToUser(dviz_core::Command::Request::PAUSE_LATER, res.response, user_id))
      {
	ROS_ERROR("[DVizCore] Error in pause_later command.");
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for pause_later (%d given, 1 required).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for pause_later (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  }
  else if(req.command.compare(dviz_core::Command::Request::RESET_ROBOT) == 0)
  {
    if(req.args.size() == 1)
    {
      int user_id = atoi(req.args[0].c_str());
      if(!passCommandToUser(dviz_core::Command::Request::RESET_ROBOT, res.response, user_id))
      {
	ROS_ERROR("[DVizCore] Error in reset_robot command.");
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for reset_robot (%d given, 1 required).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for reset_robot (" << req.args.size() << " given, 1 required).";
      res.response = ss.str();
      return false;
    }
  }
  else if(req.command.compare(dviz_core::Command::Request::LOAD_TASK) == 0)
  {
    if(req.args.size() == 2)
    {
      int user_id = atoi(req.args[0].c_str());
      std::vector<std::string> args;
      args.push_back(req.args[1]);
      if(!passCommandToUser(dviz_core::Command::Request::LOAD_TASK, res.response, user_id, args))
      {
	ROS_ERROR("[DVizCore] Error in load_task command.");
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for load_task (%d given, 2 required).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_task (" << req.args.size() << " given, 2 required).";
      res.response = ss.str();
      return false;
    }
  }
  else if(req.command.compare(dviz_core::Command::Request::LOAD_SCENE) == 0)
  {
    if(req.args.size() == 2)
    {
      int user_id = atoi(req.args[0].c_str());
      std::vector<std::string> args;
      args.push_back(req.args[1]);
      if(!passCommandToUser(dviz_core::Command::Request::LOAD_SCENE, res.response, user_id, args))
      {
	ROS_ERROR("[DVizCore] Error in load_scene command.");
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for load_scene (%d given, 2 required).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_scene (" << req.args.size() << " given, 2 required).";
      res.response = ss.str();
      return false;
    }
  }
  else if(req.command.compare(dviz_core::Command::Request::SHOW_BASE_PATH) == 0)
  {
    if(req.args.size() == 1 || req.args.size() == 2)
    {
      int user_id = atoi(req.args[0].c_str());
      std::vector<std::string> args;
      if(req.args.size() == 2)
      {
	args.push_back(req.args[1]);
      }

      if(!passCommandToUser(dviz_core::Command::Request::SHOW_BASE_PATH, res.response, user_id, args))
      {
	ROS_ERROR("[DVizCore] Error in show_base_path command.");
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for show_base_path (%d given, 1 required, 1 optional).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for show_base_path (" << req.args.size() << " given, 1 required, 1 optional).";
      res.response = ss.str();
      return false;
    }
  }
  else if(req.command.compare(dviz_core::Command::Request::LOAD_MESH) == 0)
  {
    if(req.args.size() == 3 || req.args.size() == 4)
    {
      int user_id = atoi(req.args[0].c_str());
      std::vector<std::string> args;
      args.push_back(req.args[1]);
      args.push_back(req.args[2]);
      if(req.args.size() == 4)
      {
	args.push_back(req.args[3]);
      }
      if(!passCommandToUser(dviz_core::Command::Request::LOAD_MESH, res.response, user_id, args))
      {
	ROS_ERROR("[DVizCore] Error in load_mesh command.");
      }
    }
    else
    {
      ROS_ERROR("[DVizCore] Invalid number of arguments for load_mesh (%d given, 3 required, 1 optional).",
		req.args.size());
      std::stringstream ss;
      ss << "Invalid number of arguments for load_mesh (" << req.args.size() << " given, 3 required, 1 optional).";
      res.response = ss.str();
      return false;
    }  
  }
  else if(req.command.compare(dviz_core::Command::Request::BASIC_GRIPPER) == 0)
  {
    // @todo
    ROS_WARN("[DVizCore] Not implemented!");
    // if(req.args.size() == 0)
    // {
    //   // Enable basic gripper controls.
    //   setGripperOrientationControl(false);
    // }
    // else if(req.args.size() == 1)
    // {
    //   bool basic_gripper = req.args[0].compare("true") == 0;
    //   setGripperOrientationControl(!basic_gripper);
    // }
    // else
    // {
    //   ROS_ERROR("[DVizCore] Invalid number of arguments for basic_gripper (%d given, 1 optional).",
    // 		req.args.size());
    //   std::stringstream ss;
    //   ss << "Invalid number of arguments for basic_gripper (" << req.args.size() << " given, 1 optional).";
    //   res.response = ss.str();
    //   return false;
    // }
  }
  else
  {
    ROS_ERROR("[DVizCore] Invalid command \"%s\".", req.command.c_str());
    std::stringstream ss;
    ss << "Invalid command \"" << req.command.c_str() << "\".";
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

  // Establish a command service client to issue commands to the newly created user.
  ros::NodeHandle nh;
  ROS_INFO("adding %s", resolveName("dviz_command", id).c_str());
  if(user_command_services_.find(id) != user_command_services_.end())
  {
    ROS_INFO("[DVizCore] Adding service %s.", resolveName("dviz_command", id).c_str());
    user_command_services_.insert(std::pair<int, ros::ServiceClient>(
				    id, nh.serviceClient<dviz_core::Command>(resolveName("dviz_command", id))));
  }
  else
  {
    ROS_INFO("[DVizCore] Re-adding service %s.", resolveName("dviz_command", id).c_str());
    user_command_services_[id] = nh.serviceClient<dviz_core::Command>(resolveName("dviz_command", id));
  }

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
      ROS_INFO("[DVizCore] Passed %s command to user ID %d.", command.c_str(), id);
    }
    else
    {
      ROS_ERROR("[DVizCore] Something went wrong in passing %s command to user ID %d!", command.c_str(), id);
      std::stringstream ss;
      ss << "Something went wrong in passing " << command << " command to user ID " << id << ".";
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

void DemonstrationVisualizerCore::run()
{
  ros::Rate rate(10.0);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
}

} // namespace demonstration_visualizer
