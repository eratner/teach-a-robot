#ifndef GOAL_H
#define GOAL_H

#include <string>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

#include "demonstration_visualizer/end_effector_controller.h"

namespace demonstration_visualizer {

/**
 * @brief A base class for all goal types. All goals must have a 
 *        (unique) goal number, a string description (may be empty)
 *        and a goal type.
 */
class Goal
{
public:
  enum GoalType { PICK_UP = 0,
		  PLACE };

  static const char *GoalTypeNames[];
  
  static const int NumGoalTypes;

  Goal();

  Goal(int goal_number, const std::string &description);
  
  virtual ~Goal();

  virtual std::string toString() const;

  virtual GoalType getType() const = 0;

  int getGoalNumber() const;

  void setGoalNumber(int goal_number);

  std::string getDescription() const;

  void setDescription(const std::string &description);

protected:
  int goal_number_;
  std::string description_;
  
};

class PickUpGoal : public Goal
{
public:
  PickUpGoal();

  PickUpGoal(int goal_number, 
	     const std::string &description,
	     int object_id = 0,
	     const geometry_msgs::Pose &grasp = geometry_msgs::Pose(),
             const geometry_msgs::Pose &object_pose = geometry_msgs::Pose());

  virtual ~PickUpGoal();

  GoalType getType() const;

  int getObjectID() const;

  void setObjectID(int id);

  geometry_msgs::Pose getGraspPose() const;

  void setGraspPose(const geometry_msgs::Pose &grasp);

  geometry_msgs::Pose getInitialObjectPose() const;
  
  void setInitialObjectPose(const geometry_msgs::Pose &pose);

  bool isGraspDone() const;

  void setGraspDone(bool);

  double getGraspDistance() const;

  void setGraspDistance(double);

  double getGripperJointPosition() const;

  void setGripperJointPosition(double);

private:
  int object_id_;
  geometry_msgs::Pose grasp_pose_;
  geometry_msgs::Pose initial_object_pose_;
  bool grasp_done_;
  double grasp_distance_;
  double gripper_joint_position_;

};

class PlaceGoal : public Goal
{
public:
  PlaceGoal();

  PlaceGoal(int goal_number,
	    const std::string &description,
	    int object_id = 0,
	    const geometry_msgs::Pose &pose = geometry_msgs::Pose());

  virtual ~PlaceGoal();

  GoalType getType() const;

  void setObjectID(int id);

  int getObjectID() const;

  void setPlacePose(const geometry_msgs::Pose &pose);

  geometry_msgs::Pose getPlacePose() const;

  bool ignoreYaw() const;

  void setIgnoreYaw(bool);

private:
  int object_id_;
  geometry_msgs::Pose place_pose_;
  bool ignore_yaw_;

};

} // namespace demonstration_visualizer

#endif // GOAL_H
