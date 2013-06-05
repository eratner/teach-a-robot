#include "pr2_motion_recorder.h"

PR2MotionRecorder::PR2MotionRecorder()
  : is_recording_(false), 
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

  nh.param("write_bag_path", write_bag_path_, std::string(""));
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
    ROS_INFO("Recording finished with %d joint states.", write_bag_.getSize());
    write_bag_.close();
  }

  return true;
}

bool PR2MotionRecorder::beginReplay(pr2_motion_recorder::FilePath::Request  &,
				    pr2_motion_recorder::FilePath::Response &)
{
  // @todo 
  return true;
}

bool PR2MotionRecorder::endReplay(pr2_motion_recorder::FilePath::Request  &,
				  pr2_motion_recorder::FilePath::Response &)
{
  // @todo
  return true;
}

void PR2MotionRecorder::recordJoints(const sensor_msgs::JointState &msg)
{
  if(is_recording_)
  {
    write_bag_.write("/joint_states", ros::Time::now(), msg);
  }
}

void PR2MotionRecorder::run()
{
  ros::Rate loop_rate(10.0);

  while(ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
