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

  joint_states_subscription_ = nh.subscribe("/joint_states", 
					    100,
					    &PR2MotionRecorder::recordJoints,
					    this);

  base_pose_subscription_ = nh.subscribe("/base_pose",
					 100,
					 &PR2MotionRecorder::recordBasePose,
					 this);

  set_pose_client_ = nh.serviceClient<pr2_simple_simulator::SetPose>("/set_robot_pose");

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
  ROS_INFO("Added %d poses.", poses_.size());

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
    if(pose_count_ == poses_.size() /*&& joint_position_count_ = joint_positions_.size()*/)
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

    // @todo Replay the next joint positions.
    // if(is_replaying_ && joint_position_count_ < joint_positions_.size())
    // { ... }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
