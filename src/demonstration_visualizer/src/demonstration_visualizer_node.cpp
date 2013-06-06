#include "demonstration_visualizer_node.h"

DemonstrationVisualizerNode::DemonstrationVisualizerNode(int argc, char **argv)
{
  if(!init(argc, argv))
    ROS_ERROR("Unable to connect to master!");

  interactive_marker_server_ = new interactive_markers::InteractiveMarkerServer("mesh_marker");
}

DemonstrationVisualizerNode::~DemonstrationVisualizerNode()
{
  delete interactive_marker_server_;
  interactive_marker_server_ = 0;

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
  ros::NodeHandle nh;

  // Services for communicating with the motion recording service provider.
  begin_recording_client_ = nh.serviceClient<pr2_motion_recorder::FilePath>("/motion_recorder/begin_recording");
  end_recording_client_ = nh.serviceClient<std_srvs::Empty>("/motion_recorder/end_recording");
  begin_replay_client_ = nh.serviceClient<pr2_motion_recorder::FilePath>("/motion_recorder/begin_replay");
  end_replay_client_ = nh.serviceClient<std_srvs::Empty>("/motion_recorder/end_replay");

  // Advertise topic for publishing markers.
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // Start the thread.
  start();

  return true;
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
  return begin_replay_client_.call(srv);
}

bool DemonstrationVisualizerNode::endReplay(std_srvs::Empty &srv)
{
  return end_replay_client_.call(srv);
}

void DemonstrationVisualizerNode::publishVisualizationMarker(const visualization_msgs::Marker &msg,
							     bool interactive_marker)
{
  if(interactive_marker)
  {
    ROS_INFO("Attaching a visual marker to marker %d.", msg.id);
    // Attach an interactive marker to control this marker.
    // 1. Create interactive marker.
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/odom_combined";
    int_marker.name = "test_marker";
    int_marker.description = "Simple 1-DOF Control";

    // 2. Create non-interactive marker control.
    visualization_msgs::InteractiveMarkerControl marker_control;
    marker_control.always_visible = true;
    marker_control.markers.push_back(msg);

    // 3. Add control to interactive marker.
    int_marker.controls.push_back(marker_control);

    // 4. Create a control to move the marker.
    visualization_msgs::InteractiveMarkerControl move_control_x;
    move_control_x.name = "move_x";
    move_control_x.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

    // 5. Add the control to the interactive marker.
    int_marker.controls.push_back(move_control_x);

    interactive_marker_server_->insert(int_marker, 
    				       boost::bind(
				         &DemonstrationVisualizerNode::processInteractiveMarkerFeedback,
					 this,
					 _1)
				       );
    interactive_marker_server_->applyChanges();
  }

  marker_pub_.publish(msg);
}

void DemonstrationVisualizerNode::run()
{
  ros::Rate rate(10.0);
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Shutting down ROS...");
  Q_EMIT rosShutdown();
}

void DemonstrationVisualizerNode::processInteractiveMarkerFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  ROS_INFO_STREAM(feedback->marker_name << " is now at position ("
		  << feedback->pose.position.x << ", "
		  << feedback->pose.position.y << ", "
		  << feedback->pose.position.z << ").");
}
