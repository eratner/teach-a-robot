#include "demonstration_visualizer/goal.h"

namespace demonstration_visualizer {

const char *Goal::GoalTypeNames[] = { "Pick Up", "Place" };

Goal::Goal()
  : goal_number_(0), description_("")
{

}

Goal::Goal(int goal_number, const std::string &description)
  : goal_number_(goal_number), description_(description)
{

}

Goal::~Goal()
{

}

std::string Goal::toString() const
{
  std::stringstream s;
  s << "Goal number " << getGoalNumber() << " is of type " 
    << GoalTypeNames[(int)getType()] << " and has description: " 
    << getDescription();

  return s.str();
}

int Goal::getGoalNumber() const
{
  return goal_number_;
}

void Goal::setGoalNumber(int goal_number)
{
  goal_number_ = goal_number;
}

std::string Goal::getDescription() const
{
  return description_;
}

void Goal::setDescription(const std::string &description)
{
  description_ = description;
}

PickUpGoal::PickUpGoal()
  : Goal(0, ""), object_(), pregrasp_pose_()
{

}

PickUpGoal::PickUpGoal(int goal_number, 
		       const std::string &description,
		       const visualization_msgs::Marker &object,
		       const geometry_msgs::Pose &pregrasp)
  : Goal(goal_number, description), object_(object), pregrasp_pose_(pregrasp)
{

}

PickUpGoal::~PickUpGoal()
{

}

Goal::GoalType PickUpGoal::getType() const
{
  return Goal::PICK_UP;
}

visualization_msgs::Marker PickUpGoal::getObject() const
{
  return object_;
}

void PickUpGoal::setObject(const visualization_msgs::Marker &object)
{
  object_ = object;
}

geometry_msgs::Pose PickUpGoal::getPregraspPose() const
{
  return pregrasp_pose_;
}

void PickUpGoal::setPregraspPose(const geometry_msgs::Pose &pregrasp)
{
  pregrasp_pose_ = pregrasp;
}

}
