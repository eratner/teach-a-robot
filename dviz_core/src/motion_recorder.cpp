#include <dviz_core/motion_recorder.h>

namespace demonstration_visualizer
{

MotionRecorder::MotionRecorder(int user_id)
  : is_recording_(false),
    is_replaying_(false),
    bag_count_(0), 
    write_bag_path_(""),
    write_bag_(),
    read_bag_(),
    pose_count_(0),
    joint_states_count_(0),
    user_id_(user_id),
    current_goal_(-1)
{

}

MotionRecorder::~MotionRecorder()
{

}

bool MotionRecorder::beginRecording(const std::string &path,
				    const std::string &task_name)
{
  if(!is_recording_)
  {
    is_recording_ = true;
    // Start recording to a new bag file.
    std::stringstream file_path;
    // @todo append user id to the name of the file.
    file_path << path << "/demonstration" << bag_count_ << ".bag";
    write_bag_path_ = file_path.str();

    try
    {
      write_bag_.open(file_path.str(), rosbag::bagmode::Write);
    }
    catch(const rosbag::BagException &e)
    {
      ROS_ERROR("[MotionRec] Failed to open bagfile %s: %s", file_path.str().c_str(), e.what());
      return false;
    }

    // Initialize the user demonstration message that will be written to the bagfile.
    demo_.user_id = user_id_;
    //demo_.time = ...;
    //demo_.date = ...;
    demo_.task_name = task_name;
    demo_.demo_id = bag_count_;

    demo_.steps.clear();

    bag_count_++;

    ROS_INFO("[MotionRec] Beginning to record motion to %s.", file_path.str().c_str());
  }
  else
  {
    ROS_ERROR("[MotionRec] Cannot begin recording while replaying a bagfile!");
    return false;
  }

  return true;
}

void MotionRecorder::endRecording()
{
  if(is_recording_)
  {
    is_recording_ = false;

    // Record the demonstration to a bag file.
    flush();

    // Stop recording.
    ROS_INFO("[MotionRec] Recording finished with %d messages.", write_bag_.getSize());
    write_bag_.close();

    if(!getBasePath(write_bag_path_, base_path_))
    {
      ROS_ERROR("[MotionRec] Error constructing the base path for the latest recording!");
    }
  }
}

bool MotionRecorder::beginReplay(const std::string &file)
{
  // @todo fix the base path
  // if(!getBasePath(file, base_path_))
  // {
  //   ROS_ERROR("[MotionRec] Error constructing the base path from bagfile %s!", file.c_str());
  //   return false;
  // }

  // Load appropriate bag file, and get the user demonstration data. 
  try
  {
    read_bag_.open(file, rosbag::bagmode::Read);
  }
  catch(const rosbag::BagException &e)
  {
    ROS_ERROR("[MotionRec] Failed to open bagfile %s: %s", file.c_str(), e.what());
    return false;
  }

  rosbag::View view(read_bag_, rosbag::TopicQuery("/demonstration"));
  rosbag::View::iterator iter = view.begin();
  dviz_core::UserDemonstration::ConstPtr loaded_demo = (*iter).instantiate<dviz_core::UserDemonstration>();
  if(loaded_demo == NULL)
  {
    ROS_ERROR("[MotionRec] Failed to load user demonstration from bag file!");
    return false;
  }

  poses_.clear();
  pose_count_ = 0;
  joint_states_.clear();
  joint_states_count_ = 0;

  // Populate a vector of poses and joint states from the waypoints.
  for(std::vector<dviz_core::Step>::const_iterator s_it = loaded_demo->steps.begin();
      s_it != loaded_demo->steps.end(); ++s_it)
  {
    for(std::vector<dviz_core::Waypoint>::const_iterator w_it = s_it->waypoints.begin();
	w_it != s_it->waypoints.end(); ++w_it)
    {
      poses_.push_back(w_it->base_pose);
      pose_count_++;
      joint_states_.push_back(w_it->joint_states);
      joint_states_count_++;
    }
  }

  ROS_INFO("[MotionRec] Added %d poses and %d joint states messages.", pose_count_, joint_states_count_);

  read_bag_.close();

  if(!is_replaying_)
    is_replaying_ = true;

  return true;
}

void MotionRecorder::endReplay()
{
  is_replaying_ = false;
}

bool MotionRecorder::getBasePath(const std::string &file, visualization_msgs::Marker &base_path)
{
  // @todo fix this.
  // visualization_msgs::Marker base_path;
  base_path.header.frame_id = "/map";
  base_path.header.stamp = ros::Time::now();
  base_path.ns = "motion_rec";
  base_path.action = visualization_msgs::Marker::ADD;
  base_path.pose.orientation.w = 1;
  base_path.id = 0;
  base_path.type = visualization_msgs::Marker::LINE_STRIP;
  base_path.scale.x = 0.03;
  base_path.color.r = 1;
  base_path.color.a = 0.8;

  rosbag::Bag bag;
  try
  {
    bag.open(file, rosbag::bagmode::Read);
  }
  catch(const rosbag::BagException &e)
  {
    ROS_ERROR("[MotionRec] Failed to open %s when constructing base path: %s", file.c_str(), e.what());
    return false;
  }

  // rosbag::View poses_view(bag, rosbag::TopicQuery("/base_pose"));

  // geometry_msgs::Point last;
  // int skipped = 0;

  // foreach(rosbag::MessageInstance const m, poses_view)
  // {
  //   geometry_msgs::PoseStamped::ConstPtr base_pose = m.instantiate<geometry_msgs::PoseStamped>();
  //   if(base_pose != NULL)
  //   {
  //     geometry_msgs::Point p;
  //     p.x = base_pose->pose.position.x;
  //     p.y = base_pose->pose.position.y;
  //     p.z = base_pose->pose.position.z;

  //     if(p.x == last.x && p.y == last.y && p.z == last.z)
  // 	skipped++;
  //     else
  //     {
  // 	last = p;
  // 	base_path.points.push_back(p);
  //     }
  //   }
  // }
  // ROS_INFO("[MotionRec] Added %d points to the base path (skipped %d).", base_path.points.size(),
  // 	   skipped);

  bag.close();

  // return base_path;
  return true;
}

visualization_msgs::Marker MotionRecorder::getBasePath()
{
  return base_path_;
}

void MotionRecorder::recordWaypoint(const dviz_core::Waypoint &waypoint)
{
  if(isRecording())
  {
    if(demo_.steps.size() <= current_goal_)
    {
      ROS_ERROR("[MotionRec] Not enough steps in the current demonstration!");
    }
    else
    {
      demo_.steps[current_goal_].waypoints.push_back(waypoint);
    }
  }
}

void MotionRecorder::addStep(const std::string &action,
			     const std::string &object_type,
			     const geometry_msgs::Pose &grasp)
{
  current_goal_++;
  
  dviz_core::Step step;
  step.goal_number = current_goal_;
  step.action = action;
  step.object_type = object_type;
  step.grasp = grasp;

  demo_.steps.push_back(step);
}

bool MotionRecorder::changeCurrentStep(int goal_number)
{
  if(goal_number >= 0 && goal_number < demo_.steps.size())
  {
    current_goal_ = goal_number;
  }
  else
  {
    ROS_ERROR("[MotionRec] Invalid goal number %d!", goal_number);
    return false;
  }

  return true;
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

geometry_msgs::Pose MotionRecorder::getNextBasePose()
{
  if(pose_count_ >= poses_.size())
  {
    ROS_INFO("[MotionRec] Done replaying poses.");
    return geometry_msgs::Pose();
  }
  
  pose_count_++;
  return poses_.at(pose_count_-1);
}

void MotionRecorder::flush()
{
  if(isRecording())
  {
    write_bag_.write("/demonstration", ros::Time::now(), demo_);
  }
}

} // namespace demonstration_visualizer
