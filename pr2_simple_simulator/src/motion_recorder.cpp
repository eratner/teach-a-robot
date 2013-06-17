#include "pr2_simple_simulator/motion_recorder.h"

MotionRecorder::MotionRecorder()
  : is_recording_(false),
    is_replaying_(false),
    bag_count_(0), 
    write_bag_path_(""),
    write_bag_(),
    read_bag_(),
    pose_count_(0),
    joint_states_count_(0)
{

}

MotionRecorder::~MotionRecorder()
{

}

void MotionRecorder::beginRecording(const std::string &path)
{
  if(!is_recording_)
  {
    is_recording_ = true;
    // Start recording to a new bag file.
    std::stringstream file_path;
    file_path << path << "/motion" << bag_count_ << ".bag";
    bag_count_++;
    write_bag_.open(file_path.str(), rosbag::bagmode::Write);
    ROS_INFO("[MotionRec] Beginning to record motion to %s.", file_path.str().c_str());
  }
}

void MotionRecorder::endRecording()
{
  if(is_recording_)
  {
    is_recording_ = false;
    // Stop recording.
    ROS_INFO("[MotionRec] Recording finished with %d messages.", write_bag_.getSize());
    write_bag_.close();
  }
}

void MotionRecorder::beginReplay(const std::string &file)
{
  base_path_ = getBasePath(file);

  // Load appropriate bag file, and populate a vector of PoseStamped messages to 
  // send to the robot simulator.
  read_bag_.open(file, rosbag::bagmode::Read);

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
  ROS_INFO("[MotionRec] Added %d poses.", poses_.size());

  // Populate a vector of joint states to send to the robot simulator.
  rosbag::View joints_view(read_bag_, rosbag::TopicQuery("/joint_states"));
  joint_states_.clear();
  joint_states_count_ = 0;
  
  foreach(rosbag::MessageInstance const m, joints_view)
  {
    sensor_msgs::JointState::ConstPtr joint_state = m.instantiate<sensor_msgs::JointState>();
    if(joint_state != NULL)
    {
      joint_states_.push_back(*joint_state);
    }
  }
  ROS_INFO("[MotionRec] Added %d joints states.", joint_states_.size());

  read_bag_.close();

  if(!is_replaying_)
    is_replaying_ = true;
}

void MotionRecorder::endReplay()
{
  is_replaying_ = false;
}

visualization_msgs::Marker MotionRecorder::getBasePath(const std::string &file)
{
  visualization_msgs::Marker base_path;
  base_path.header.frame_id = "/map";
  base_path.header.stamp = ros::Time::now();
  base_path.ns = "motion_rec";
  base_path.action = visualization_msgs::Marker::ADD;
  base_path.pose.orientation.w = 1;
  base_path.id = 0;
  base_path.type = visualization_msgs::Marker::LINE_STRIP;
  base_path.scale.x = 0.2;
  base_path.color.r = 1;
  base_path.color.a = 1;

  rosbag::Bag bag;
  bag.open(file, rosbag::bagmode::Read);

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
  ROS_INFO("[MotionRec] Added %d points to the base path.", base_path.points.size());

  bag.close();

  return base_path;
}

visualization_msgs::Marker MotionRecorder::getBasePath() const
{
  return base_path_;
}

void MotionRecorder::recordJoints(const sensor_msgs::JointState &msg)
{
  if(is_recording_)
  {
    write_bag_.write("/joint_states", ros::Time::now(), msg);
  }
}

void MotionRecorder::recordBasePose(const geometry_msgs::PoseStamped &msg)
{
  if(is_recording_)
  {
    write_bag_.write("/base_pose", ros::Time::now(), msg);
  }
}

bool MotionRecorder::isRecording() const
{
  return is_recording_;
}

bool MotionRecorder::isReplaying() const
{
  return is_replaying_;
}

int MotionRecorder::getJointsRemaining() const
{
  return (joint_states_.size() - joint_states_count_);
}

int MotionRecorder::getPosesRemaining() const
{
  return (poses_.size() - pose_count_);
}

sensor_msgs::JointState MotionRecorder::getNextJoints()
{
  if(joint_states_count_ >= joint_states_.size())
  {
    ROS_INFO("[MotionRec] Done replaying joint states.");
    return sensor_msgs::JointState();
  }

  joint_states_count_++;
  return joint_states_.at(joint_states_count_-1);
}

geometry_msgs::PoseStamped MotionRecorder::getNextBasePose()
{
  if(pose_count_ >= poses_.size())
  {
    ROS_INFO("[MotionRec] Done replaying poses.");
    return geometry_msgs::PoseStamped();
  }
  
  pose_count_++;
  return poses_.at(pose_count_-1);
}
