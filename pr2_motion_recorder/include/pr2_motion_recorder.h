#ifndef PR2_MOTION_RECORDER_H
#define PR2_MOTION_RECORDER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <pr2_motion_recorder/FilePath.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>

#include <string>
#include <sstream>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajectoryClient;

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

  void recordBasePose(const geometry_msgs::PoseWithCovarianceStamped &msg);

  void run();

private:
  void startJointTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal,
			    TrajectoryClient *trajectory_client);

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

  TrajectoryClient *r_arm_traj_client_;

};

#endif // PR2_MOTION_RECORDER_H
