#ifndef BASE_MOVEMENT_CONTROLLER_H
#define BASE_MOVEMENT_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <cmath>

static const char *STATE_NAMES[] = { "INITIAL",
                                     "READY",
				     "ROTATE_TO_POSITION",
				     "TRANSLATE_TO_POSITION",
				     "ROTATE_TO_ORIENTATION",
				     "DONE" };

class BaseMovementController
{
public:
  enum State { INITIAL = 0,
	       READY,
               ROTATE_TO_POSITION,
	       TRANSLATE_TO_POSITION,
	       ROTATE_TO_ORIENTATION,
	       DONE };

  BaseMovementController();

  virtual ~BaseMovementController();

  geometry_msgs::Twist getNextVelocities(const geometry_msgs::Pose &current_pose,
					 const geometry_msgs::Pose &goal_pose);

  void setState(State state);

  State getState() const;

private:
  State last_state_;
  int frames_;
  
  // States.
  geometry_msgs::Twist rotateToGoalPosition(double current_angle,
					    double angle_to_goal);

  geometry_msgs::Twist translateToGoalPosition(double distance);

  geometry_msgs::Twist rotateToGoalOrientation(const geometry_msgs::Quaternion &current_orientation,
					       const geometry_msgs::Quaternion &goal_orientation);

  geometry_msgs::Twist done();

  geometry_msgs::Twist initial();

  void printStateTransition(State next_state);

};

#endif // BASE_MOVEMENT_CONTROLLER_H
