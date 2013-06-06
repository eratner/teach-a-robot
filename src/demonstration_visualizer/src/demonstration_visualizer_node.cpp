#include "demonstration_visualizer_node.h"

DemonstrationVisualizerNode::DemonstrationVisualizerNode(int argc, char **argv)
{
  if(!init(argc, argv))
    ROS_ERROR("Unable to connect to master!");
}

DemonstrationVisualizerNode::~DemonstrationVisualizerNode()
{
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

void DemonstrationVisualizerNode::publishVisualizationMarker(const visualization_msgs::Marker &msg)
{
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
