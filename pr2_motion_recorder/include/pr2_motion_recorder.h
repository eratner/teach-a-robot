#ifndef PR2_MOTION_RECORDER_H
#define PR2_MOTION_RECORDER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <pr2_motion_recorder/FilePath.h>
#include <pr2_simple_simulator/SetPose.h>
#include <pr2_simple_simulator/SetJoints.h>
#include <geometry_msgs/PoseStamped.h>

#include <string>
#include <sstream>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

class PR2MotionRecorder
{
public:
  PR2MotionRecorder();

  virtual ~PR2MotionRecorder();

  bool beginRecording(pr2_motion_recorder::FilePath::Request  &,
		      pr2_motion_recorder::FilePath::Response &);

  bool endRecording(std_srvs::Empty::Request  &,
		    std_srvs::Empty::Response &);

  bool beginReplay(pr2_motion_recorder::FilePath::Request  &,
		   pr2_motion_recorder::FilePath::Response &);

  bool endReplay(std_srvs::Empty::Request  &,
		 std_srvs::Empty::Response &);

  void recordJoints(const sensor_msgs::JointState &msg);

  void recordBasePose(const geometry_msgs::PoseStamped &msg);

  void run();

private:
  bool is_recording_;
  bool is_replaying_;
  int bag_count_;
  std::string write_bag_path_;
  rosbag::Bag write_bag_;
  rosbag::Bag read_bag_;

  ros::ServiceServer begin_rec_service_;
  ros::ServiceServer end_rec_service_;
  ros::ServiceServer begin_replay_service_;
  ros::ServiceServer end_replay_service_;
  
  ros::Subscriber joint_states_subscription_;
  ros::Subscriber base_pose_subscription_;

  ros::ServiceClient set_pose_client_;
  ros::ServiceClient set_joint_positions_client_;

  std::vector<geometry_msgs::PoseStamped> poses_;
  int pose_count_;

  std::vector<pr2_simple_simulator::SetJoints> joint_positions_;
  int joint_position_count_;

};

#endif // PR2_MOTION_RECORDER_H
