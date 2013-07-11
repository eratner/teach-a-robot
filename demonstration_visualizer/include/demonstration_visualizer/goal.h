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
	     const visualization_msgs::Marker &object = visualization_msgs::Marker(),
	     const geometry_msgs::Pose &pregrasp = geometry_msgs::Pose());

  virtual ~PickUpGoal();

  GoalType getType() const;

  visualization_msgs::Marker getObject() const;

  void setObject(const visualization_msgs::Marker &object);

  geometry_msgs::Pose getPregraspPose() const;

  void setPregraspPose(const geometry_msgs::Pose &pregrasp);

  bool isPregraspDone() const;

  void setPregraspDone(bool);

  double getPregraspDistance() const;

  void setPregraspDistance(double);

private:
  visualization_msgs::Marker object_;
  geometry_msgs::Pose pregrasp_pose_;
  bool pregrasp_done_;
  double pregrasp_distance_;

};

/*
class PlaceGoal : public Goal
{
  // @todo
};
*/

} // namespace demonstration_visualizer

#endif // GOAL_H
