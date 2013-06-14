#include "demonstration_visualizer/demonstration_visualizer_node.h"

DemonstrationVisualizerNode::DemonstrationVisualizerNode(int argc, char **argv)
{
  if(!init(argc, argv))
    ROS_ERROR("[DVizNode] Unable to connect to master!");

  demonstration_scene_manager_ = new DemonstrationSceneManager();
  edit_goals_mode_ = true;
  current_goal_ = 0;

  interactive_marker_server_ = new interactive_markers::InteractiveMarkerServer("mesh_marker");
}

DemonstrationVisualizerNode::~DemonstrationVisualizerNode()
{
  delete demonstration_scene_manager_;
  delete interactive_marker_server_;

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
  begin_recording_client_ = nh.serviceClient<pr2_motion_recorder::FilePath>("/pr2_motion_rec/begin_recording");
  end_recording_client_ = nh.serviceClient<std_srvs::Empty>("/pr2_motion_rec/end_recording");
  begin_replay_client_ = nh.serviceClient<pr2_motion_recorder::FilePath>("/pr2_motion_rec/begin_replay");
  end_replay_client_ = nh.serviceClient<std_srvs::Empty>("/pr2_motion_rec/end_replay");

  // Service for resetting the state of the robot and environment.
  reset_robot_client_ = nh.serviceClient<std_srvs::Empty>("/reset_robot");
  // Service for setting the speed of the robot.
  set_robot_speed_client_ = nh.serviceClient<pr2_simple_simulator::SetSpeed>("/set_speed");

  // Advertise topic for publishing markers.
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // Start the thread.
  start();

  return true;
}

std::string DemonstrationVisualizerNode::getGlobalFrame() const
{
  return global_frame_;
}

bool DemonstrationVisualizerNode::beginRecording(pr2_motion_recorder::FilePath &srv)
{
  return begin_recording_client_.call(srv);
}

bool DemonstrationVisualizerNode::endRecording(std_srvs::Empty &srv)
{
  return end_recording_client_.call(srv);
}

bool DemonstrationVisualizerNode::beginReplay(pr2_motion_recorder::FilePath &srv)
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

void DemonstrationVisualizerNode::publishVisualizationMarker(const visualization_msgs::Marker &msg,
							     bool attach_interactive_marker)
{
  if(attach_interactive_marker)
  {
    ROS_INFO("[DVizNode] Attaching an interactive marker to visual marker %d.", msg.id);

    // Attach an interactive marker to control this marker.
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = global_frame_;
    int_marker.pose = msg.pose;

    // Give each interactive marker a unique name according to each mesh's unique id.
    std::stringstream marker_name;
    marker_name << "mesh_marker_" << msg.id;

    int_marker.name = marker_name.str();
    int_marker.description = "Move Mesh";

    // Add a non-interactive control for the mesh.
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(msg);

    int_marker.controls.push_back(control);

    // Attach a 6-DOF control for moving the mesh around.
    attach6DOFControl(int_marker);

    interactive_marker_server_->insert(int_marker,
				       boost::bind(
                                         &DemonstrationVisualizerNode::processInteractiveMarkerFeedback,
					 this,
					 _1)
				       );
    interactive_marker_server_->applyChanges();
  }
  else
  {
    marker_pub_.publish(msg);
  }
}

bool DemonstrationVisualizerNode::removeInteractiveMarker(const std::string &name)
{
  ROS_INFO("Removing interactive marker %s.", name.c_str());
  bool success = interactive_marker_server_->erase(name);
  if(success)
    interactive_marker_server_->applyChanges();
  
  return success;
}

void DemonstrationVisualizerNode::clearInteractiveMarkers()
{
  interactive_marker_server_->clear();
  interactive_marker_server_->applyChanges();
}

void DemonstrationVisualizerNode::run()
{
  ros::Rate rate(10.0);
  while(ros::ok())
  {
    updateTaskGoals();

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("[DVizNode] Shutting down ROS...");
  Q_EMIT rosShutdown();
}

void DemonstrationVisualizerNode::processInteractiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  Q_EMIT interactiveMarkerFeedback(feedback);
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

void DemonstrationVisualizerNode::updateTaskGoals()
{
  if(getSceneManager()->getNumGoals() == 0)
    return;

  if(current_goal_ >= getSceneManager()->getNumGoals())
  {
    ROS_INFO("[DVizNode] All goals complete!");
    return;
  }

  if(edit_goals_mode_ && !getSceneManager()->goalsChanged())
    return;

  // Draw each of the goals in the current task. 
  if(edit_goals_mode_)
  {
    std::vector<visualization_msgs::Marker> goals = getSceneManager()->getGoals();
    std::vector<visualization_msgs::Marker>::iterator it;
    for(it = goals.begin(); it != goals.end(); ++it)
    {
      // Attach an interactive marker to control this marker.
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = global_frame_;
      int_marker.pose = it->pose;

      // Give each interactive marker a unique name according to each mesh's unique id.
      std::stringstream marker_name;
      marker_name << "goal_marker_" << it->id;

      int_marker.name = marker_name.str();

      std::stringstream marker_desc;
      marker_desc << "Goal " << it->id;
      int_marker.description = marker_desc.str();

      // Add a non-interactive control for the mesh.
      visualization_msgs::InteractiveMarkerControl control;
      control.always_visible = true;
      control.markers.push_back(*it);

      int_marker.controls.push_back(control);

      // Attach a 6-DOF control for moving the mesh around.
      attach6DOFControl(int_marker);

      interactive_marker_server_->insert(int_marker,
					 boost::bind(&DemonstrationVisualizerNode::processGoalFeedback,
						     this,
						     _1));
      interactive_marker_server_->applyChanges();
    }
  }
  else // Otherwise, just draw the current goal.
  {
    // Clear the existing goal markers.
    std::vector<visualization_msgs::Marker> goals = getSceneManager()->getGoals();
    std::vector<visualization_msgs::Marker>::iterator it;
    for(it = goals.begin(); it != goals.end(); ++it)
    {
      // Get the name of this marker according to its id.
      std::stringstream marker_name;
      marker_name << "goal_marker_" << it->id;

      interactive_marker_server_->erase(marker_name.str());
    }
    interactive_marker_server_->applyChanges();

    // Draw the current goal.
    if(getSceneManager()->getNumGoals() > 0)
    {
      //ROS_INFO("Getting goal %d.", current_goal_);
      marker_pub_.publish(getSceneManager()->getGoal(current_goal_));
    }
  }

  getSceneManager()->setGoalsChanged(false);
}

void DemonstrationVisualizerNode::processGoalFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  int i;
  for(i = feedback->marker_name.size()-1; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  if(!getSceneManager()->moveGoal(atoi(feedback->marker_name.substr(i+1).c_str()),
				  feedback->pose))
  {
    ROS_ERROR("[DVizNode] Demonstration scene manager failed to update task goal pose!");
  }  

  getSceneManager()->setGoalsChanged(true);
}

DemonstrationSceneManager *DemonstrationVisualizerNode::getSceneManager()
{
  return demonstration_scene_manager_;
}

void DemonstrationVisualizerNode::interactiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  int i;
  for(i = feedback->marker_name.size()-1; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  if(!getSceneManager()->updateMeshPose(atoi(feedback->marker_name.substr(i+1).c_str()),
					feedback->pose))
  {
    ROS_ERROR("[DVizNode] Demonstration scene manager failed to update pose of mesh!");
  }
}

void DemonstrationVisualizerNode::setEditGoalsMode(bool mode)
{
  edit_goals_mode_ = mode;
}

void DemonstrationVisualizerNode::setCurrentGoal(int goal_number)
{
  if(goal_number < 0 || goal_number >= getSceneManager()->getNumGoals())
  {
    ROS_ERROR("[DVizNode] Invalid goal number!");
    return;
  }

  current_goal_ = goal_number;
}
