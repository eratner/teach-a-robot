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
#include <geometry_msgs/Pose.h>
#include <visualization_msgs/Marker.h>

#include <dviz_core/UserDemonstration.h>
#include <dviz_core/Step.h>
#include <dviz_core/Waypoint.h>

#include <string>
#include <sstream>

#include <boost/date_time/posix_time/posix_time.hpp>

namespace demonstration_visualizer
{

class MotionRecorder
{
public:
  static const std::string DEFAULT_DEMONSTRATION_PATH;

  MotionRecorder(int user_id = 0);

  virtual ~MotionRecorder();

  /**
   * @brief Creates a bagfile for recording motion at the specified 
   *        file path. 
   * @param path The path indicating where to save the bagfile.
   * @param task_name (optional) The name of the task that is being recorded.
   * @param start_goal (optional) The goal number to begin with. 
   */
  bool beginRecording(const std::string &path, const std::string &task_name = "");

  void endRecording();

  bool beginReplay(const std::string &file);

  void endReplay();

  /**
   * @brief Creates a line strip marker to visualize the path that the
   *        base of the robot took throughout the recorded motion.
   * @param[in] file Specifies the path to the recorded bag file.
   * @param[out] base_path The constructed path of the base.
   */
  bool getBasePath(const std::string &file, visualization_msgs::Marker &base_path);

  visualization_msgs::Marker getBasePath();

  void recordWaypoint(const dviz_core::Waypoint &waypoint);

  void addStep(const std::string &action, const std::string &object_type, const geometry_msgs::Pose &grasp);

  bool changeCurrentStep(int goal_number);

  bool isRecording() const;

  bool isReplaying() const;

  int getJointsRemaining() const;
  
  int getPosesRemaining() const;

  /**
   * @brief If replaying from a bagfile, this will return the next
   *        set of joint angle positions in the recorded trajectory.
   */
  sensor_msgs::JointState getNextJoints();

  /**
   * @brief If replaying from a bagfile, this will return the next
   *        base pose from the recorded trajectory.
   */
  geometry_msgs::Pose getNextBasePose();

private:
  void flush();

  bool is_recording_;
  bool is_replaying_;
  int bag_count_;
  std::string write_bag_path_;
  rosbag::Bag write_bag_;
  rosbag::Bag read_bag_;

  int user_id_;
  int current_goal_;

  dviz_core::UserDemonstration demo_;

  std::vector<geometry_msgs::Pose> poses_;
  int pose_count_;

  std::vector<sensor_msgs::JointState> joint_states_;
  int joint_states_count_;

  visualization_msgs::Marker base_path_;
  
};

} // namespace demonstration_visualizer

#endif // MOTION_RECORDER_H
