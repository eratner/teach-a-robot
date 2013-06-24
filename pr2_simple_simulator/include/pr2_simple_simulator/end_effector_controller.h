#ifndef END_EFFECTOR_CONTROLLER_H
#define END_EFFECTOR_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_datatypes.h>
#include <cmath>

static const char *END_EFFECTOR_STATE_NAMES[] = { "INITIAL",
                                                  "READY",
						  "MOVING_TO_GOAL",
						  "INVALID_GOAL",
						  "DONE" };
						
class EndEffectorController
{
public:
  enum State { INITIAL = 0,
               READY,
	       MOVING_TO_GOAL,
	       INVALID_GOAL,
	       DONE };
	       
  EndEffectorController();

  virtual ~EndEffectorController();

  geometry_msgs::Twist moveTo(const geometry_msgs::Pose &current,
			      const geometry_msgs::Pose &goal);

  void setState(State state);

  State getState() const;

  State getLastState() const;

  void setSpeed(double speed);
  
  double getSpeed() const;

private:
  State current_state_;
  State last_state_;
  int frames_;
  double speed_;

  // States
  geometry_msgs::Twist initial();

  geometry_msgs::Twist movingToGoal(const geometry_msgs::Pose &current,
				    const geometry_msgs::Pose &goal);

  geometry_msgs::Twist invalidGoal();

  geometry_msgs::Twist done();

  void printStateTransition(State next_state);
};

#endif // END_EFFECTOR_CONTROLLER_H
