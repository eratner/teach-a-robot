#include "demonstration_visualizer/demonstration_visualizer_node.h"

namespace demonstration_visualizer {

DemonstrationVisualizerNode::DemonstrationVisualizerNode(int argc, char **argv)
{
  if(!init(argc, argv))
    ROS_ERROR("[DVizNode] Unable to connect to master!");

  recorder_ = new MotionRecorder();

  pviz_ = new PViz();

  int_marker_server_ = new interactive_markers::InteractiveMarkerServer("dviz_interactive_markers");

  //collision_checker_ = new CollisionChecker();

  object_manager_ = new ObjectManager(/*collision_checker_*/);

  demonstration_scene_manager_ = new DemonstrationSceneManager(int_marker_server_, /*collision_checker_, */object_manager_);

  simulator_ = new PR2Simulator(recorder_, pviz_, int_marker_server_, object_manager_);

  // Start the thread.
  start();
}

DemonstrationVisualizerNode::~DemonstrationVisualizerNode()
{
  delete simulator_;
  delete demonstration_scene_manager_;
  delete recorder_;
  delete pviz_;


  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

bool DemonstrationVisualizerNode::init(int argc, char **argv)
{
  ros::init(argc, argv, "demonstration_visualizer");

  // Make sure that we can communicate with the master.
  if(!ros::master::check())
    return false;

  ros::start();

  ros::NodeHandle nh("~");

  nh.param("world_frame", world_frame_, std::string("/map"));

  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);

  // Advertise topic for publishing end-effector velocity commands.
  end_effector_vel_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/end_effector_vel_cmd", 1);

  end_effector_marker_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/end_effector_marker_vel", 1);

  base_vel_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/vel_cmd", 1);

  return true;
}

std::string DemonstrationVisualizerNode::getWorldFrame() const
{
  return world_frame_;
}

void DemonstrationVisualizerNode::pauseSimulator()
{
  return simulator_->pause();
}

void DemonstrationVisualizerNode::playSimulator()
{
  return simulator_->play();
}

void DemonstrationVisualizerNode::run()
{
  ros::Rate rate(10.0);
  while(ros::ok())
  {
    simulator_->run();

    getSceneManager()->updateScene();

    // Check to see if the end effector has reached a goal.
    if(!getSceneManager()->editGoalsMode() || 
       !getSceneManager()->taskDone() || 
       getSceneManager()->getNumGoals() != 0)
    {
      if(getSceneManager()->hasReachedGoal(getSceneManager()->getCurrentGoal(), getEndEffectorPose()) &&
	 getSceneManager()->hasReachedGoal(getSceneManager()->getCurrentGoal(), getEndEffectorMarkerPose()))
      {
	ROS_INFO("[DVizNode] Reached goal %d!", getSceneManager()->getCurrentGoal());
	getSceneManager()->setCurrentGoal(getSceneManager()->getCurrentGoal() + 1);
	getSceneManager()->setGoalsChanged(true);

	Q_EMIT goalComplete(getSceneManager()->getCurrentGoal() - 1);
      }
    }

    // Focus the camera according to the position of the end-effector and the 
    // current goal.
    if(!getSceneManager()->taskDone() && getSceneManager()->getNumGoals() > 0)
    {
      Q_EMIT updateCamera(getEndEffectorPose(), getSceneManager()->getCurrentGoalPose());
    }
    else
    {
      // @todo sort of a hack, should make this cleaner.
      Q_EMIT updateCamera(getEndEffectorPose(), getEndEffectorPose());
    }

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("[DVizNode] Shutting down ROS...");
  Q_EMIT rosShutdown();
}

void DemonstrationVisualizerNode::setRobotSpeed(double linear, double angular)
{
  simulator_->setSpeed(linear, angular);
}

void DemonstrationVisualizerNode::resetRobot()
{
  simulator_->resetRobot();
}

DemonstrationSceneManager *DemonstrationVisualizerNode::getSceneManager()
{
  return demonstration_scene_manager_;
}

MotionRecorder *DemonstrationVisualizerNode::getMotionRecorder()
{
  return recorder_;
}

interactive_markers::InteractiveMarkerServer *DemonstrationVisualizerNode::getInteractiveMarkerServer()
{
  return int_marker_server_;
}

void DemonstrationVisualizerNode::processKeyEvent(int key, int type)
{
  // Pass along the key event to the simulator.
  simulator_->processKeyEvent(key, type);

  switch(key)
  {
  case Qt::Key_Up:
    {
      // Move the end-effector in the positive z-direction.
      if(type == QEvent::KeyPress)
      {
	geometry_msgs::Twist vel;
	vel.linear.x = vel.linear.y = 0;
	vel.linear.z = 0.1; 
	end_effector_marker_vel_pub_.publish(vel);
      }
      else if(type == QEvent::KeyRelease)
      {
	geometry_msgs::Twist vel;
	vel.linear.x = vel.linear.y = vel.linear.z = 0;
	end_effector_marker_vel_pub_.publish(vel);
      }
      break;
    }
  case Qt::Key_Down:
    {
      // Move the end-effector in the negative z-direction.
      if(type == QEvent::KeyPress)
      {
	geometry_msgs::Twist vel;
	vel.linear.x = vel.linear.y = 0;
	vel.linear.z = -0.1;
	end_effector_marker_vel_pub_.publish(vel);
      }
      else if(type == QEvent::KeyRelease)
      {
	geometry_msgs::Twist vel;
	vel.linear.x = vel.linear.y = vel.linear.z = 0;
	end_effector_marker_vel_pub_.publish(vel);
      }
      break;
    }
  default:
    break;
  }
}

geometry_msgs::Pose DemonstrationVisualizerNode::getBasePose()
{
  return simulator_->getBasePose();
}

geometry_msgs::Pose DemonstrationVisualizerNode::getEndEffectorPose()
{
  return simulator_->getEndEffectorPose();
}

geometry_msgs::Pose DemonstrationVisualizerNode::getEndEffectorMarkerPose()
{
  return simulator_->getEndEffectorMarkerPose();
}

void DemonstrationVisualizerNode::sendBaseCommand(const geometry_msgs::Pose &pose)
{
  simulator_->setRobotBaseCommand(pose);
}

void DemonstrationVisualizerNode::sendBaseVelocityCommand(const geometry_msgs::Twist &cmd)
{
  base_vel_cmd_pub_.publish(cmd);
}

void DemonstrationVisualizerNode::setJointStates(const sensor_msgs::JointState &joints)
{
  simulator_->setJointStates(joints);
}

void DemonstrationVisualizerNode::showBasePath(const std::string &filename)
{
  visualization_msgs::Marker base_path;

  if(filename.empty())
  {
    base_path = recorder_->getBasePath();
  }
  else
  {
    base_path = recorder_->getBasePath(filename);
  }
  
  marker_pub_.publish(base_path);
}

void DemonstrationVisualizerNode::showInteractiveGripper(int goal_number)
{
  geometry_msgs::Pose gripper_pose = getSceneManager()->getPregraspPose(goal_number);

  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = "/map";
  std::stringstream s;
  s << "pregrasp_marker_goal_" << goal_number;
  int_marker.name = s.str();
  int_marker.description = "";
  int_marker.pose = gripper_pose;

  visualization_msgs::InteractiveMarkerControl control;
  std::vector<visualization_msgs::Marker> markers;
  geometry_msgs::Pose origin; 
  // @todo compute this based on the goal object.
  origin.position.x = -0.25;
  origin.position.y = origin.position.z = 0;
  origin.orientation.x = origin.orientation.y = origin.orientation.z = 0;
  origin.orientation.w = 1;
  pviz_->getGripperMeshesMarkerMsg(origin, 0.2, "pr2_simple_sim", 1, true, markers);

  for(int i = 0; i < markers.size(); ++i)
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
			       &DemonstrationVisualizerNode::pregraspMarkerFeedback,
			       this,
			       _1)
			     );
  int_marker_server_->applyChanges();
}

void DemonstrationVisualizerNode::pregraspMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  ROS_INFO("[DVizNode] Moving pregrasp interactive marker %s.", feedback->marker_name.c_str());

  int i = feedback->marker_name.size()-1;
  for(; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  getSceneManager()->setPregraspPose(atoi(feedback->marker_name.substr(i+1).c_str()),
				     feedback->pose);
}

} // namespace demonstration_visualizer
