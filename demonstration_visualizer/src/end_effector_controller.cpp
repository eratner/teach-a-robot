#include "demonstration_visualizer/end_effector_controller.h"

namespace demonstration_visualizer {

EndEffectorController::EndEffectorController()
 : last_state_(INITIAL), current_state_(INITIAL), frames_(0), speed_(0.05)
{

}

EndEffectorController::~EndEffectorController()
{

}

geometry_msgs::Twist EndEffectorController::moveTo(const geometry_msgs::Pose &current,
						   const geometry_msgs::Pose &goal)
{
  frames_++;

  double distance = std::sqrt(std::pow(goal.position.x - current.position.x, 2) +
			      std::pow(goal.position.y - current.position.y, 2) + 
			      std::pow(goal.position.z - current.position.z, 2));

  switch(current_state_)
  {
  case INITIAL:
    return initial();
  case READY:
  case MOVING_TO_GOAL:
    {
      if(distance < 0.02)
	return done();
      else
	return movingToGoal(current, goal);
    }
  case INVALID_GOAL:
    return invalidGoal();
  case DONE:
    return done();
  default:
    ROS_ERROR("EndEffectorController error!");
    break;
  }
}

void EndEffectorController::setState(State state)
{
  printStateTransition(state);

  current_state_ = state;
}

EndEffectorController::State EndEffectorController::getState() const
{
  return current_state_;
}

EndEffectorController::State EndEffectorController::getLastState() const
{
  return last_state_;
}

void EndEffectorController::setSpeed(double speed)
{
  speed_ = speed;
}

double EndEffectorController::getSpeed() const
{
  return speed_;
}

geometry_msgs::Twist EndEffectorController::initial()
{
  printStateTransition(INITIAL);

  geometry_msgs::Twist vel;
  vel.linear.x = 0;
  vel.linear.y = 0;
  vel.linear.z = 0;

  return vel;
}

geometry_msgs::Twist EndEffectorController::movingToGoal(const geometry_msgs::Pose &current,
							 const geometry_msgs::Pose &goal)
{
  printStateTransition(MOVING_TO_GOAL);

  geometry_msgs::Vector3 r;
  r.x = goal.position.x - current.position.x;
  r.y = goal.position.y - current.position.y;
  r.z = goal.position.z - current.position.z;

  double length = std::sqrt(r.x*r.x + r.y*r.y + r.z*r.z);
  r.x = (r.x/length) * speed_;
  r.y = (r.y/length) * speed_;
  r.z = (r.z/length) * speed_;

  geometry_msgs::Twist vel;
  vel.linear.x = r.x;
  vel.linear.y = r.y;
  vel.linear.z = r.z;

  //ROS_INFO("velocity = (%f, %f, %f)", vel.linear.x, vel.linear.y, vel.linear.z);

  return vel;
}

geometry_msgs::Twist EndEffectorController::invalidGoal()
{
  printStateTransition(INVALID_GOAL);

  geometry_msgs::Twist vel;
  vel.linear.x = vel.linear.y = vel.linear.z = 0;

  return vel;
}

geometry_msgs::Twist EndEffectorController::done()
{
  printStateTransition(DONE);

  geometry_msgs::Twist vel;
  vel.linear.x = vel.linear.y = vel.linear.z = 0;

  return vel;
}

void EndEffectorController::printStateTransition(State next_state)
{
  if(next_state != current_state_)
  {
    ROS_INFO("Transitioning to state %s after %d frames in state %s.",
	     END_EFFECTOR_STATE_NAMES[next_state],
	     frames_,
	     END_EFFECTOR_STATE_NAMES[current_state_]);
    last_state_ = current_state_;
    current_state_ = next_state;
    frames_ = 0;
  }
}

} // namespace demonstration_visualizer
