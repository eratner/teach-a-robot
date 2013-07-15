#include "demonstration_visualizer/goal.h"

namespace demonstration_visualizer {

const char *Goal::GoalTypeNames[] = { "Pick Up", "Place" };

const int Goal::NumGoalTypes = 2;

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
  : Goal(0, ""), object_(), grasp_pose_(), grasp_done_(false), grasp_distance_(0.25)
{

}

PickUpGoal::PickUpGoal(int goal_number, 
		       const std::string &description,
		       int object_id,
		       const visualization_msgs::Marker &object,
		       const geometry_msgs::Pose &grasp)
  : Goal(goal_number, description), object_id_(object_id),
    object_(object), grasp_pose_(grasp), 
    grasp_done_(false), grasp_distance_(0.25)
{

}

PickUpGoal::~PickUpGoal()
{

}

Goal::GoalType PickUpGoal::getType() const
{
  return Goal::PICK_UP;
}

int PickUpGoal::getObjectID() const
{
  return object_id_;
}

void PickUpGoal::setObjectID(int id)
{
  object_id_ = id;
}

visualization_msgs::Marker PickUpGoal::getObject() const
{
  return object_;
}

void PickUpGoal::setObject(const visualization_msgs::Marker &object)
{
  object_ = object;
}

geometry_msgs::Pose PickUpGoal::getGraspPose() const
{
  return grasp_pose_;
}

void PickUpGoal::setGraspPose(const geometry_msgs::Pose &grasp)
{
  grasp_pose_ = grasp;
}

bool PickUpGoal::isGraspDone() const
{
  return grasp_done_;
}

void PickUpGoal::setGraspDone(bool done)
{
  grasp_done_ = done;
}

double PickUpGoal::getGraspDistance() const
{
  return grasp_distance_;
}

void PickUpGoal::setGraspDistance(double distance)
{
  grasp_distance_ = distance;
}

}
