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

  nh.param("global_frame", global_frame_, std::string("/map"));

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
  simulator_->setRobotBaseCommand(pose);
}

void DemonstrationVisualizerNode::sendBaseVelocityCommand(const geometry_msgs::Twist &cmd)
{
  base_vel_cmd_pub_.publish(cmd);
}

void DemonstrationVisualizerNode::updateEndEffectorMarkerPose(const geometry_msgs::Pose &pose)
{
  end_effector_marker_pose_ = pose;
}

void DemonstrationVisualizerNode::setJointStates(const sensor_msgs::JointState &joints)
{
  simulator_->setJointStates(joints);
}

} // namespace demonstration_visualizer
