#include "demonstration_visualizer_node.h"

DemonstrationVisualizerNode::DemonstrationVisualizerNode(int argc, char **argv)
{
  if(!init(argc, argv))
    ROS_ERROR("Unable to connect to master!");

  interactive_marker_server_ = new interactive_markers::InteractiveMarkerServer("mesh_marker");
  
  // Create an interactive marker at the base of the PR2, so that the user can move it.
  base_marker_server_ = new interactive_markers::InteractiveMarkerServer("base_marker");
  
  visualization_msgs::InteractiveMarker int_marker;
  int_marker.header.frame_id = global_frame_;
  int_marker.name = "base_marker";
  int_marker.description = "Move Base";

  // geometry_msgs::PoseStamped base_marker_pose;
  // base_marker_pose.pose = latest_base_pose_.pose;
  // base_marker_pose.header.frame_id = global_frame_;
  // base_marker_pose.header.stamp = ros::Time();
  // geometry_msgs::Vector3 scale;
  // scale.x = 1.2;
  // scale.y = 1.2;
  // scale.z = 1.2;

  // std_msgs::ColorRGBA color;
  // color.r = 0.5;
  // color.g = 0.0;
  // color.b = 0.5;
  // color.a = 0.5;

  // visualization_msgs::Marker base_marker = makeMeshMarker("package://pr2_description/meshes/base_v0/base.dae",
  // 							  "demonstration_visualizer",
  // 							  0, // Reserved id = 0 for base marker.
  // 							  base_marker_pose,
  // 							  scale,
  // 							  0.5,
  // 							  true);

  // visualization_msgs::InteractiveMarkerControl base_control;
  // base_control.always_visible = true;
  // base_control.markers.push_back(base_marker);

  // int_marker.controls.push_back(base_control);

  visualization_msgs::InteractiveMarkerControl marker_control;
  marker_control.orientation.w = 1;
  marker_control.orientation.x = 0;
  marker_control.orientation.y = 1;
  marker_control.orientation.z = 0;
  //marker_control.always_visible = true;
  //marker_control.markers.push_back(base_marker);
  marker_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;

  // @todo add planar rotation control.

  int_marker.controls.push_back(marker_control);
  
  base_marker_server_->insert(int_marker,
			      boost::bind(&DemonstrationVisualizerNode::processBaseMarkerFeedback,
					  this,
					  _1)
			      );
  base_marker_server_->applyChanges();
}

DemonstrationVisualizerNode::~DemonstrationVisualizerNode()
{
  delete interactive_marker_server_;
  delete base_marker_server_;

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

  // Advertise topic for publishing markers.
  marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // Subscribe to the base pose ground truth.
  base_pose_sub_ = nh.subscribe("/amcl_pose", 
				100,
				&DemonstrationVisualizerNode::updateLatestBasePose,
				this);

  // Advertise topic for driving the base of the robot.
  base_cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

  base_goal_pose_.pose = latest_base_pose_.pose;

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
    ROS_INFO("Attaching an interactive marker to visual marker %d.", msg.id);

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
    moveBaseToGoal();

    ros::spinOnce();
    rate.sleep();
  }
  ROS_INFO("Shutting down ROS...");
  Q_EMIT rosShutdown();
}

void DemonstrationVisualizerNode::updateLatestBasePose(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = msg.header;
  pose_stamped.pose = msg.pose.pose;
  latest_base_pose_ = pose_stamped;
  updateBaseMarker();
}

void DemonstrationVisualizerNode::processBaseMarkerFeedback(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
)
{
  switch(feedback->event_type)
  {
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
    break;
  case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
    {
      base_movement_controller_.setState(BaseMovementController::READY);
      base_goal_pose_.pose = feedback->pose;
      // ROS_INFO("Frame %s", feedback->header.frame_id.c_str());
      // base_goal_pose_.pose.position.x = feedback->mouse_point.x;
      // base_goal_pose_.pose.position.y = feedback->mouse_point.y;
      double yaw = std::atan2(base_goal_pose_.pose.position.y - latest_base_pose_.pose.position.y,
			      base_goal_pose_.pose.position.x - latest_base_pose_.pose.position.x);
      ROS_INFO("New goal set at (x, y, yaw) = (%f, %f, %f).", 
      	       base_goal_pose_.pose.position.x,
      	       base_goal_pose_.pose.position.y,
      	       yaw * (180.0/M_PI));
      geometry_msgs::PoseStamped base_pose_stamped;
      base_pose_stamped.header.frame_id = global_frame_;
      base_pose_stamped.header.stamp = ros::Time();
      base_pose_stamped.pose = base_goal_pose_.pose;
      geometry_msgs::Vector3 scale;
      scale.x = scale.y = scale.z = 1.0;
      std_msgs::ColorRGBA color;
      color.r = 0.0;
      color.g = 0.0;
      color.b = 1.0;
      color.a = 0.5;
      visualization_msgs::Marker goal_marker = makeShapeMarker(visualization_msgs::Marker::SPHERE,
							       "demonstration_visualizer",
							       1, // Reserved for the goal marker.
							       base_pose_stamped,
							       scale,
							       color);

      marker_pub_.publish(goal_marker);
    }
  default:
    break;
  }
}

void DemonstrationVisualizerNode::processInteractiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  Q_EMIT interactiveMarkerFeedback(feedback);
}

void DemonstrationVisualizerNode::updateBaseMarker()
{ 
  // After the robot has completed moving to the goal pose, snap the marker back to the
  // base pose of the robot.
  if(base_movement_controller_.getState() == BaseMovementController::DONE)
  {
    base_marker_server_->setPose("base_marker", latest_base_pose_.pose, latest_base_pose_.header);
    base_marker_server_->applyChanges();
  }
}

void DemonstrationVisualizerNode::moveBaseToGoal()
{
  if(base_movement_controller_.getState() != BaseMovementController::DONE)
  {
    geometry_msgs::Twist base_cmd;

    base_cmd = base_movement_controller_.getNextVelocities(latest_base_pose_.pose, base_goal_pose_.pose);

    base_cmd_vel_pub_.publish(base_cmd);
  }
}
