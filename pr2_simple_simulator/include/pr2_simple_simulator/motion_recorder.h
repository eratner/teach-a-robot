/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef MOTION_RECORDER_H
#define MOTION_RECORDER_H

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <sstream>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH

class MotionRecorder
{
public:
  MotionRecorder();

  virtual ~MotionRecorder();

  /**
   * @brief Creates a bagfile for recording motion at the specified 
   *        file path. 
   */
  void beginRecording(const std::string &path);

  void endRecording();

  void beginReplay(const std::string &file);

  void endReplay();

  visualization_msgs::Marker getBasePath(const std::string &file);

  void recordJoints(const sensor_msgs::JointState &msg);

  void recordBasePose(const geometry_msgs::PoseStamped &msg);

  bool isRecording() const;

  bool isReplaying() const;

  int getJointsRemaining() const;
  
  int getPosesRemaining() const;

  /**
   * @brief If replaying from a bagfile, this will return the next
   *        set of joint angle positions in the recorded trajectory.
   */
  sensor_msgs::JointState getNextJoints();

  geometry_msgs::PoseStamped getNextBasePose();

private:
  bool is_recording_;
  bool is_replaying_;
  int bag_count_;
  std::string write_bag_path_;
  rosbag::Bag write_bag_;
  rosbag::Bag read_bag_;

  std::vector<geometry_msgs::PoseStamped> poses_;
  int pose_count_;

  std::vector<sensor_msgs::JointState> joint_states_;
  int joint_states_count_;
  
};

#endif // PR2_MOTION_RECORDER_H