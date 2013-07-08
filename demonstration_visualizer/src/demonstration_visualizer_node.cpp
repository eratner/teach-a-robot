#include "demonstration_visualizer/demonstration_visualizer_node.h"

namespace demonstration_visualizer {

DemonstrationVisualizerNode::DemonstrationVisualizerNode(int argc, char **argv)
{
  if(!init(argc, argv))
    ROS_ERROR("[DVizNode] Unable to connect to master!");

  demonstration_scene_manager_ = new DemonstrationSceneManager();

  // Start the thread.
  start();
}

DemonstrationVisualizerNode::~DemonstrationVisualizerNode()
{
  delete demonstration_scene_manager_;

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

  nh.param("global_frame", global_frame_, std::string("/map"));

  // Services for communicating with the motion recording service provider.
  begin_recording_client_ = nh.serviceClient<pr2_simple_simulator::FilePath>("/motion_recorder/begin_recording");
  end_recording_client_ = nh.serviceClient<std_srvs::Empty>("/motion_recorder/end_recording");
  begin_replay_client_ = nh.serviceClient<pr2_simple_simulator::FilePath>("/motion_recorder/begin_replay");
  end_replay_client_ = nh.serviceClient<std_srvs::Empty>("/motion_recorder/end_replay");

  // Services for playing/pausing the simulation.
  pause_simulator_client_ = nh.serviceClient<std_srvs::Empty>("/pause");
  play_simulator_client_ = nh.serviceClient<std_srvs::Empty>("/play");

  // Service for resetting the state of the robot and environment.
  reset_robot_client_ = nh.serviceClient<std_srvs::Empty>("/reset_robot");
  // Service for setting the speed of the robot.
  set_robot_speed_client_ = nh.serviceClient<pr2_simple_simulator::SetSpeed>("/set_speed");

  // Service for passing keyboard events to the simulator.
  key_event_client_ = nh.serviceClient<pr2_simple_simulator::KeyEvent>("/key_event");

  // Service for visualizing the path of the base in a recorded motion.
  show_base_path_client_ = nh.serviceClient<pr2_simple_simulator::FilePath>("/show_base_path");

  // Service for setting the base command (i.e. the location of the carrot marker.)
  set_base_command_client_ = nh.serviceClient<pr2_simple_simulator::SetPose>("/set_base_command");

  // Advertise topic for publishing end-effector velocity commands.
  end_effector_vel_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/end_effector_vel_cmd", 1);

  end_effector_marker_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/end_effector_marker_vel", 1);

  base_vel_cmd_pub_ = nh.advertise<geometry_msgs::Twist>("/vel_cmd", 1);

  // Subscribe to the pose of the (right) end effector.
  end_effector_pose_sub_ = nh.subscribe("/end_effector_pose", 10, 
					&DemonstrationVisualizerNode::updateEndEffectorPose,
					this);

  // Subscribe to the pose of the base in the map frame.
  base_pose_sub_ = nh.subscribe("/base_pose", 10, 
				&DemonstrationVisualizerNode::updateBasePose,
				this);

  end_effector_marker_pose_sub_ = nh.subscribe("/end_effector_marker_pose", 10,
					       &DemonstrationVisualizerNode::updateEndEffectorMarkerPose,
					       this);

  return true;
}

std::string DemonstrationVisualizerNode::getGlobalFrame() const
{
  return global_frame_;
}

bool DemonstrationVisualizerNode::beginRecording(pr2_simple_simulator::FilePath &srv)
{
  return begin_recording_client_.call(srv);
}

bool DemonstrationVisualizerNode::endRecording(std_srvs::Empty &srv)
{
  return end_recording_client_.call(srv);
}

bool DemonstrationVisualizerNode::beginReplay(pr2_simple_simulator::FilePath &srv)
{
  std_srvs::Empty empty;
  if(!reset_robot_client_.call(empty))
  {
    ROS_ERROR("[DVizNode] Error resetting the world!");
    return false;
  }

  return begin_replay_client_.call(srv);
}

bool DemonstrationVisualizerNode::endReplay(std_srvs::Empty &srv)
{
  return end_replay_client_.call(srv);
}

bool DemonstrationVisualizerNode::pauseSimulator(std_srvs::Empty &srv)
{
  return pause_simulator_client_.call(srv);
}

bool DemonstrationVisualizerNode::playSimulator(std_srvs::Empty &srv)
{
  return play_simulator_client_.call(srv);
}

void DemonstrationVisualizerNode::run()
{
  ros::Rate rate(10.0);
  while(ros::ok())
  {
    getSceneManager()->updateScene();

    // Focus the camera according to the position of the end-effector and the 
    // current goal.
    if(!getSceneManager()->taskDone() && getSceneManager()->getNumGoals() > 0)
    {
      geometry_msgs::Pose current_goal_pose = 
	getSceneManager()->getGoal(getSceneManager()->getCurrentGoal()).pose;
      Q_EMIT updateCamera(end_effector_pose_, current_goal_pose);
    }
    else
    {
      // @todo sort of a hack, should make this cleaner.
      Q_EMIT updateCamera(end_effector_pose_, end_effector_pose_);
    }

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("[DVizNode] Shutting down ROS...");
  Q_EMIT rosShutdown();
}

void DemonstrationVisualizerNode::setRobotSpeed(double linear, double angular)
{
  pr2_simple_simulator::SetSpeed speed;
  speed.request.linear = linear;
  speed.request.angular = angular;

  if(!set_robot_speed_client_.call(speed))
  {
    ROS_ERROR("[DVizNode] Error setting the robot speed!");
  }
}

void DemonstrationVisualizerNode::resetRobot()
{
  std_srvs::Empty empty;
  
  if(!reset_robot_client_.call(empty))
  {
    ROS_ERROR("[DVizNode] Failed to reset the robot!");
  }
}

DemonstrationSceneManager *DemonstrationVisualizerNode::getSceneManager()
{
  return demonstration_scene_manager_;
}

void DemonstrationVisualizerNode::updateEndEffectorPose(const geometry_msgs::PoseStamped &pose)
{
  end_effector_pose_ = pose.pose;

  if(getSceneManager()->editGoalsMode() || 
     getSceneManager()->taskDone() || 
     getSceneManager()->getNumGoals() == 0)
    return;

  // ROS_INFO("marker = (%f, %f, %f)", end_effector_marker_pose_.position.x,
  // 	   end_effector_marker_pose_.position.y, end_effector_marker_pose_.position.z);

  if(getSceneManager()->hasReachedGoal(getSceneManager()->getCurrentGoal(), pose.pose) &&
     getSceneManager()->hasReachedGoal(getSceneManager()->getCurrentGoal(), end_effector_marker_pose_))
  {
    ROS_INFO("[DVizNode] Reached goal %d!", getSceneManager()->getCurrentGoal());
    getSceneManager()->setCurrentGoal(getSceneManager()->getCurrentGoal() + 1);
    getSceneManager()->setGoalsChanged(true);

    Q_EMIT goalComplete(getSceneManager()->getCurrentGoal() - 1);
  }
}

void DemonstrationVisualizerNode::processKeyEvent(int key, int type)
{
  // Pass along the key event to the simulator.
  pr2_simple_simulator::KeyEvent event;
  if(type == QEvent::KeyPress)
    event.request.type = pr2_simple_simulator::KeyEvent::Request::KEY_PRESS;
  else if(type == QEvent::KeyRelease)
    event.request.type = pr2_simple_simulator::KeyEvent::Request::KEY_RELEASE;
  else
    event.request.type = -1;
  event.request.key = key;
  if(!key_event_client_.call(event))
  {
    ROS_ERROR("[DVizNode] Error sending key event to simulator!");
  }

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

void DemonstrationVisualizerNode::showBasePath(const std::string &filename)
{
  pr2_simple_simulator::FilePath path;
  path.request.file_path = filename;
  if(!show_base_path_client_.call(path))
  {
    ROS_ERROR("[DVizNode] Failed to show base path!");
  }
}

void DemonstrationVisualizerNode::updateBasePose(const geometry_msgs::PoseStamped &pose)
{
  base_pose_ = pose.pose;

  // ROS_INFO("[DVizNode] Updated base orientation to %f.",
  // 	   tf::getYaw(base_pose_.orientation));
}

geometry_msgs::Pose DemonstrationVisualizerNode::getBasePose() const
{
  return base_pose_;
}

void DemonstrationVisualizerNode::sendBaseCommand(const geometry_msgs::Pose &pose)
{
  pr2_simple_simulator::SetPose set;
  set.request.pose.pose = pose;
  set.request.pose.header.frame_id = "/map";
  if(!set_base_command_client_.call(set))
  {
    ROS_ERROR("[DVizNode] Failed to set the base command!");
  }
}

void DemonstrationVisualizerNode::sendBaseVelocityCommand(const geometry_msgs::Twist &cmd)
{
  base_vel_cmd_pub_.publish(cmd);
}

void DemonstrationVisualizerNode::updateEndEffectorMarkerPose(const geometry_msgs::Pose &pose)
{
  end_effector_marker_pose_ = pose;
}

} // namespace demonstration_visualizer
