#include "demonstration_visualizer/demonstration_visualizer_node.h"

namespace demonstration_visualizer {

DemonstrationVisualizerNode::DemonstrationVisualizerNode(int argc, char **argv)
{
  if(!init(argc, argv))
    ROS_ERROR("[DVizNode] Unable to connect to master!");

  demonstration_scene_manager_ = new DemonstrationSceneManager();

  recorder_ = new MotionRecorder();

  simulator_ = new PR2Simulator(recorder_);

  // Start the thread.
  start();
}

DemonstrationVisualizerNode::~DemonstrationVisualizerNode()
{
  delete demonstration_scene_manager_;
  delete recorder_;
  delete simulator_;

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
      geometry_msgs::Pose current_goal_pose = 
	getSceneManager()->getGoal(getSceneManager()->getCurrentGoal()).marker_.pose;
      Q_EMIT updateCamera(getEndEffectorPose(), current_goal_pose);
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

void DemonstrationVisualizerNode::showInteractiveGripper(const geometry_msgs::Pose &goal_pose,
							 double distance,
							 bool shadow)
{
  visualization_msgs::InteractiveMarker gripper_goal_marker = 
    simulator_->createInteractiveGripper(goal_pose, distance);

  // @todo visualize the gripper.
}

} // namespace demonstration_visualizer
