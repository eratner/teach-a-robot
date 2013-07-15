#ifndef GOAL_H
#define GOAL_H

#include <string>
#include <sstream>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>

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
	     const visualization_msgs::Marker &object = visualization_msgs::Marker(),
	     const geometry_msgs::Pose &grasp = geometry_msgs::Pose());

  virtual ~PickUpGoal();

  GoalType getType() const;

  int getObjectID() const;

  void setObjectID(int id);

  visualization_msgs::Marker getObject() const;

  void setObject(const visualization_msgs::Marker &object);

  geometry_msgs::Pose getGraspPose() const;

  void setGraspPose(const geometry_msgs::Pose &grasp);

  bool isGraspDone() const;

  void setGraspDone(bool);

  double getGraspDistance() const;

  void setGraspDistance(double);

private:
  visualization_msgs::Marker object_;
  int object_id_;
  geometry_msgs::Pose grasp_pose_;
  bool grasp_done_;
  double grasp_distance_;

};

/*
class PlaceGoal : public Goal
{
  // @todo
};
*/

} // namespace demonstration_visualizer

#endif // GOAL_H
