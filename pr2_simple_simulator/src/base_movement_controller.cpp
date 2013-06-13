#include "pr2_simple_simulator/base_movement_controller.h"

BaseMovementController::BaseMovementController()
  : last_state_(INITIAL), frames_(0),
    linear_speed_(0.2), angular_speed_(0.2)
{

}

BaseMovementController::~BaseMovementController()
{

}

geometry_msgs::Twist BaseMovementController::getNextVelocities(const geometry_msgs::Pose &current_pose,
							       const geometry_msgs::Pose &goal_pose)
{
  frames_++;
  // Compute the distance between the current and goal poses to decide which 
  // state to transition to.
  double distance = std::sqrt(std::pow(goal_pose.position.x - current_pose.position.x, 2)
			      + std::pow(goal_pose.position.y - current_pose.position.y, 2));
  double current_angle = tf::getYaw(current_pose.orientation);
  double angle_to_goal = std::atan2(goal_pose.position.y - current_pose.position.y,
				    goal_pose.position.x - current_pose.position.x);
  double goal_yaw      = tf::getYaw(goal_pose.orientation);

  // Based on the previous state and the current information, decide which state to 
  // transition to.
  switch(last_state_)
  {
  case READY:
  case ROTATE_TO_POSITION:
    {
      if(std::abs(angle_to_goal - current_angle) > 0.1 && distance >= 0.1)
	return rotateToGoalPosition(current_angle, angle_to_goal);
      else
	return translateToGoalPosition(distance);
    }
  case TRANSLATE_TO_POSITION:
    {
      if(distance < 0.1)
	return rotateToGoalOrientation(current_pose.orientation, goal_pose.orientation);
      else
      {
	if(std::abs(angle_to_goal - current_angle) > 0.2)
	  return rotateToGoalPosition(current_angle, angle_to_goal);
	else
	  return translateToGoalPosition(distance);
      }
    }
  case ROTATE_TO_ORIENTATION:
    {
      if(std::abs(goal_yaw - current_angle) < 0.1)
	return done();
      else
	return rotateToGoalOrientation(current_pose.orientation, goal_pose.orientation);
    }
  case DONE:
    return done();
  case INITIAL:
    return initial();
  default:
    ROS_ERROR("BaseMovementController error!");
    break;
  }
}

void BaseMovementController::setState(State state)
{
  printStateTransition(state);

  last_state_ = state;
}

BaseMovementController::State BaseMovementController::getState() const
{
  return last_state_;
}

void BaseMovementController::setLinearSpeed(double linear)
{
  linear_speed_ = linear;
}

double BaseMovementController::getLinearSpeed() const
{
  return linear_speed_;
}

void BaseMovementController::setAngularSpeed(double angular)
{
  angular_speed_ = angular;
}

double BaseMovementController::getAngularSpeed() const
{
  return angular_speed_;
}

geometry_msgs::Twist BaseMovementController::rotateToGoalPosition(double current_angle,
								  double angle_to_goal)
{
  printStateTransition(ROTATE_TO_POSITION);

  geometry_msgs::Twist vel;
  // Pure rotation.
  vel.linear.x = vel.linear.y = 0.0;
  vel.angular.z = angular_speed_;

  if(angle_to_goal - current_angle < 0) // Rotate counter-clockwise.
  {
    vel.angular.z *= -1.0;
  }

  return vel;
}

geometry_msgs::Twist BaseMovementController::translateToGoalPosition(double distance)
{
  printStateTransition(TRANSLATE_TO_POSITION);

  geometry_msgs::Twist vel;

  vel.angular.z = vel.linear.y = 0.0;

  // Move straight towards the goal.
  vel.linear.x = linear_speed_;

  return vel;
}

geometry_msgs::Twist BaseMovementController::rotateToGoalOrientation(
    const geometry_msgs::Quaternion &current_orientation,
    const geometry_msgs::Quaternion &goal_orientation
  )
{
  printStateTransition(ROTATE_TO_ORIENTATION);

  double current_angle = tf::getYaw(current_orientation);
  double goal_angle    = tf::getYaw(goal_orientation);

  geometry_msgs::Twist vel;
  vel.linear.x = vel.linear.y = 0.0;
  vel.angular.z = angular_speed_;

  if(goal_angle - current_angle < 0) // Rotate counter-clockwise.
  {
    vel.angular.z *= -1.0;
  }

  return vel;
}

geometry_msgs::Twist BaseMovementController::done()
{
  printStateTransition(DONE);

  geometry_msgs::Twist vel;
  
  vel.linear.x = vel.linear.y = vel.angular.z = 0.0;

  return vel;
}

geometry_msgs::Twist BaseMovementController::initial()
{
  printStateTransition(INITIAL);

  geometry_msgs::Twist vel;

  vel.linear.x = vel.linear.y = vel.angular.z = 0.0;

  return vel;
}

void BaseMovementController::printStateTransition(State next_state)
{
  if(next_state != last_state_)
  {
    ROS_INFO("Transitioning to state %s after %d frames in state %s.",
	     STATE_NAMES[next_state],
	     frames_,
	     STATE_NAMES[last_state_]);
    last_state_ = next_state;
    frames_ = 0;
  }
}
