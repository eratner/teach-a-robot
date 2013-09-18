#include <dviz_core/demonstration_visualizer_user.h>

namespace demonstration_visualizer
{

DemonstrationVisualizerUser::DemonstrationVisualizerUser(int argc, char **argv, int id)
  : id_(id), ok_(true)
{
  ROS_INFO("[DVizUser%d] Constructing user.", id_);

  if(!init(argc, argv))
  {
    ROS_ERROR("[DVizUser%d] Unable to connect to ROS master!", id_);
  }

  int_marker_server_ = new interactive_markers::InteractiveMarkerServer(resolveName("interactive_markers", id_));
  pviz_ = new PViz();
  recorder_ = new MotionRecorder();

  std::string larm_filename;
  std::string rarm_filename;
  ros::NodeHandle nh("~");
  nh.param<std::string>("left_arm_description_file", larm_filename, "");
  nh.param<std::string>("right_arm_description_file", rarm_filename, "");
  object_manager_ = new ObjectManager(rarm_filename, larm_filename);
  simulator_ = new PR2Simulator(recorder_, pviz_, int_marker_server_, object_manager_, id_);
}

DemonstrationVisualizerUser::~DemonstrationVisualizerUser()
{
  ROS_INFO("[DVizUser%d] Destructing user.", id_);

  delete int_marker_server_;
  delete object_manager_;
  delete recorder_;
  delete simulator_;
}

void DemonstrationVisualizerUser::run()
{
  ros::Rate rate(10.0);
  while(ros::ok() && ok_)
  {
    // Run the simulator.
    simulator_->run();

    // Update the scene.
    // scene_manager_->updateScene();

    // Update goals.
    // updateGoals();

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
  }
  else if(req.command.compare(dviz_core::Command::Request::PLAY) == 0)
  {
    simulator_->play();
  }
  else if(req.command.compare(dviz_core::Command::Request::PAUSE_NOW) == 0)
  {
    simulator_->pause();
  }
  else if(req.command.compare(dviz_core::Command::Request::PAUSE_LATER) == 0)
  {
    simulator_->pauseLater();
  }
  else if(req.command.compare(dviz_core::Command::Request::RESET_ROBOT) == 0)
  {
    simulator_->resetRobot();
  }
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
					 boost::bind(&DemonstrationVisualizerUser::processCommand,
						     this, _1, _2));

  return true;
}

} // namespace demonstration_visualizer
