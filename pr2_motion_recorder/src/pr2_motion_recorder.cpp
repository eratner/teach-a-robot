#include "pr2_motion_recorder.h"

PR2MotionRecorder::PR2MotionRecorder()
  : is_recording_(false),
    is_replaying_(false),
    bag_count_(0), 
    write_bag_path_(""),
    write_bag_(),
    read_bag_()
{
  ros::NodeHandle nh("~");

  begin_rec_service_ = nh.advertiseService("begin_recording", 
					   &PR2MotionRecorder::beginRecording,
					   this);

  end_rec_service_ = nh.advertiseService("end_recording",
					 &PR2MotionRecorder::endRecording,
					 this);

  begin_replay_service_ = nh.advertiseService("begin_replay",
					      &PR2MotionRecorder::beginReplay,
					      this);

  end_replay_service_ = nh.advertiseService("end_replay",
					    &PR2MotionRecorder::endReplay,
					    this);

  get_base_path_service_ = nh.advertiseService("get_base_path",
					       &PR2MotionRecorder::getBasePath,
					       this);

  joint_states_subscription_ = nh.subscribe("/joint_states", 
					    100,
					    &PR2MotionRecorder::recordJoints,
					    this);

  base_pose_subscription_ = nh.subscribe("/base_pose",
					 100,
					 &PR2MotionRecorder::recordBasePose,
					 this);

  set_pose_client_ = nh.serviceClient<pr2_simple_simulator::SetPose>("/set_robot_pose");

  set_joint_positions_client_ = nh.serviceClient<pr2_simple_simulator::SetJoints>("/set_joints");

}

PR2MotionRecorder::~PR2MotionRecorder()
{

}

bool PR2MotionRecorder::beginRecording(pr2_motion_recorder::FilePath::Request  &req,
				       pr2_motion_recorder::FilePath::Response &res)
{
  if(!is_recording_)
  {
    is_recording_ = true;
    // Start recording to a new bag file.
    std::stringstream path;
    path << req.file_path << "/motion" << bag_count_ << ".bag";
    bag_count_++;
    write_bag_.open(path.str(), rosbag::bagmode::Write);
    ROS_INFO("Beginning to record motion to %s.", path.str().c_str());
  }

  return true;
}

bool PR2MotionRecorder::endRecording(std_srvs::Empty::Request  &req,
				     std_srvs::Empty::Response &res)
{
  if(is_recording_)
  {
    is_recording_ = false;
    // Stop recording.
    ROS_INFO("Recording finished with %d messages.", write_bag_.getSize());
    write_bag_.close();
  }

  return true;
}

bool PR2MotionRecorder::beginReplay(pr2_motion_recorder::FilePath::Request  &req,
				    pr2_motion_recorder::FilePath::Response &res)
{
  // Load appropriate bag file, and populate a vector of PoseStamped messages to 
  // send to the robot simulator.
  read_bag_.open(req.file_path, rosbag::bagmode::Read);

  rosbag::View poses_view(read_bag_, rosbag::TopicQuery("/base_pose"));
  poses_.clear();
  pose_count_ = 0;

  foreach(rosbag::MessageInstance const m, poses_view)
  {
    geometry_msgs::PoseStamped::ConstPtr base_pose = m.instantiate<geometry_msgs::PoseStamped>();
    if(base_pose != NULL)
    {
      poses_.push_back(*base_pose);
    }
  }
  ROS_INFO("[PR2MotionRec] Added %d poses.", poses_.size());

  // Populate a vector of joint positions to send to the robot simulator.
  rosbag::View joints_view(read_bag_, rosbag::TopicQuery("/joint_states"));
  joint_positions_.clear();
  joint_position_count_ = 0;
  
  foreach(rosbag::MessageInstance const m, joints_view)
  {
    sensor_msgs::JointState::ConstPtr joint_state = m.instantiate<sensor_msgs::JointState>();
    if(joint_state != NULL)
    {
      pr2_simple_simulator::SetJoints joints;
      for(int i = 0; i < joint_state->name.size(); ++i)
      {
	joints.request.name.push_back(joint_state->name[i]);
	joints.request.position.push_back(joint_state->position[i]);
      }
      joint_positions_.push_back(joints);
    }
  }
  ROS_INFO("[PR2MotionRec] Added %d joints positions.", joint_positions_.size());

  read_bag_.close();

  if(!is_replaying_)
    is_replaying_ = true;

  return true;
}

bool PR2MotionRecorder::endReplay(std_srvs::Empty::Request  &,
				  std_srvs::Empty::Response &)
{
  is_replaying_ = false;

  return true;
}

bool PR2MotionRecorder::getBasePath(pr2_motion_recorder::BasePath::Request  &req,
				    pr2_motion_recorder::BasePath::Response &res)
{
  visualization_msgs::Marker base_path;
  base_path.header.frame_id = "/map";
  base_path.header.stamp = ros::Time::now();
  base_path.ns = "pr2_motion_rec";
  base_path.action = visualization_msgs::Marker::ADD;
  base_path.pose.orientation.w = 1;
  base_path.id = 0;
  base_path.type = visualization_msgs::Marker::LINE_STRIP;
  base_path.scale.x = 0.1;
  base_path.color.r = 1;
  base_path.color.a = 1;

  rosbag::Bag bag;
  bag.open(req.file_path, rosbag::bagmode::Read);

  rosbag::View poses_view(bag, rosbag::TopicQuery("/base_pose"));

  foreach(rosbag::MessageInstance const m, poses_view)
  {
    geometry_msgs::PoseStamped::ConstPtr base_pose = m.instantiate<geometry_msgs::PoseStamped>();
    if(base_pose != NULL)
    {
      geometry_msgs::Point p;
      p.x = base_pose->pose.position.x;
      p.y = base_pose->pose.position.y;
      p.z = base_pose->pose.position.z;

      base_path.points.push_back(p);
    }
  }
  ROS_INFO("[PR2MotionRec] Added %d points to the base path.", base_path.points.size());

  res.base_path = base_path;

  return true;
}

void PR2MotionRecorder::recordJoints(const sensor_msgs::JointState &msg)
{
  if(is_recording_)
  {
    write_bag_.write("/joint_states", ros::Time::now(), msg);
  }
}

void PR2MotionRecorder::recordBasePose(const geometry_msgs::PoseStamped &msg)
{
  if(is_recording_)
  {
    write_bag_.write("/base_pose", ros::Time::now(), msg);
  }
}

void PR2MotionRecorder::run()
{
  ros::Rate loop_rate(10.0);

  while(ros::ok())
  {
    if(pose_count_ == poses_.size() && 
       joint_position_count_ == joint_positions_.size() && 
       is_replaying_)
    {
      is_replaying_ = false;
      ROS_INFO("[PR2MotionRec] Done replaying.");
    }

    // Replay the next pose.
    if(is_replaying_ && pose_count_ < poses_.size())
    {
      pr2_simple_simulator::SetPose set_pose;
      set_pose.request.pose = poses_.at(pose_count_);
      pose_count_++;
      if(!set_pose_client_.call(set_pose))
	ROS_ERROR("[PR2MotionRec] Error replaying pose %d!", pose_count_-1);
    }

    // Replay the next joint positions.
    if(is_replaying_ && joint_position_count_ < joint_positions_.size())
    {
      if(!set_joint_positions_client_.call(joint_positions_.at(joint_position_count_)))
	ROS_ERROR("[PR2MotionRec] Error replaying joints positions %d!", joint_position_count_);
      joint_position_count_++;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
