#include <dviz_core/goal.h>

namespace demonstration_visualizer
{

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
  : Goal(0, ""), grasp_pose_(), grasp_done_(false), grasp_distance_(0.25)
{

}

PickUpGoal::PickUpGoal(int goal_number, 
		       const std::string &description,
		       int object_id,
		       const geometry_msgs::Pose &grasp,
		       const geometry_msgs::Pose &object_pose)
  : Goal(goal_number, description), object_id_(object_id),
    grasp_pose_(grasp), initial_object_pose_(object_pose), 
    grasp_done_(false), grasp_distance_(0.25), 
    gripper_joint_position_(EndEffectorController::GRIPPER_OPEN_ANGLE)
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

geometry_msgs::Pose PickUpGoal::getGraspPose() const
{
  return grasp_pose_;
}

void PickUpGoal::setGraspPose(const geometry_msgs::Pose &grasp)
{
  grasp_pose_ = grasp;
}

geometry_msgs::Pose PickUpGoal::getInitialObjectPose() const
{
  return initial_object_pose_;
}

void PickUpGoal::setInitialObjectPose(const geometry_msgs::Pose &pose)
{
  initial_object_pose_ = pose;
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

double PickUpGoal::getGripperJointPosition() const
{
  return gripper_joint_position_;
}

void PickUpGoal::setGripperJointPosition(double position)
{
  gripper_joint_position_ = position;
}

PlaceGoal::PlaceGoal()
  : Goal(0, ""), object_id_(0), ignore_yaw_(false)
{

}

PlaceGoal::PlaceGoal(int goal_number, const std::string &description, 
		     int object_id, const geometry_msgs::Pose &pose)
  : Goal(goal_number, description), object_id_(object_id), place_pose_(pose), ignore_yaw_(false)
{

}

PlaceGoal::~PlaceGoal()
{

}

Goal::GoalType PlaceGoal::getType() const
{
  return Goal::PLACE;
}

void PlaceGoal::setObjectID(int id)
{
  object_id_ = id;
}

int PlaceGoal::getObjectID() const
{
  return object_id_;
}

void PlaceGoal::setPlacePose(const geometry_msgs::Pose &pose)
{
  place_pose_ = pose;
}

geometry_msgs::Pose PlaceGoal::getPlacePose() const
{
  return place_pose_;
}

bool PlaceGoal::ignoreYaw() const
{
  return ignore_yaw_;
}

void PlaceGoal::setIgnoreYaw(bool ignore)
{
  ignore_yaw_ = ignore;
}

}
