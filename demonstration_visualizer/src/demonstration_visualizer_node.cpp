#include "demonstration_visualizer/demonstration_visualizer_node.h"

DemonstrationVisualizerNode::DemonstrationVisualizerNode(int argc, char **argv)
{
  if(!init(argc, argv))
    ROS_ERROR("[DVizNode] Unable to connect to master!");

  interactive_marker_server_ = new interactive_markers::InteractiveMarkerServer("mesh_marker");
}

DemonstrationVisualizerNode::~DemonstrationVisualizerNode()
{
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
  begin_recording_client_ = nh.serviceClient<pr2_motion_recorder::FilePath>("/motion_recorder/begin_recording");
  end_recording_client_ = nh.serviceClient<std_srvs::Empty>("/motion_recorder/end_recording");
  begin_replay_client_ = nh.serviceClient<pr2_motion_recorder::FilePath>("/motion_recorder/begin_replay");
  end_replay_client_ = nh.serviceClient<std_srvs::Empty>("/motion_recorder/end_replay");

  // Service for resetting the state of the robot and environment. @todo change this
  reset_robot_client_ = nh.serviceClient<std_srvs::Empty>("/reset_robot");
  // Service for setting the velocity of the robot.
  set_robot_velocity_client_ = nh.serviceClient<pr2_simple_simulator::SetVelocity>("/set_vel");

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

    // Give each interactive marker a unique name according to each mesh's unique id.
    std::stringstream marker_name;
    marker_name << "mesh_marker_" << msg.id;

    int_marker.name = marker_name.str();
    int_marker.description = "Move Mesh";

    visualization_msgs::InteractiveMarkerControl marker_control;
    marker_control.always_visible = true;
    marker_control.markers.push_back(msg);

    int_marker.controls.push_back(marker_control);

    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker.controls.push_back(control);

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

void DemonstrationVisualizerNode::setRobotVelocity(double lin_vel, double ang_vel)
{
  pr2_simple_simulator::SetVelocity vel;
  vel.request.linear = lin_vel;
  vel.request.angular = ang_vel;

  if(!set_robot_velocity_client_.call(vel))
  {
    ROS_ERROR("[DVizNode] Error setting the robot velocity!");
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
