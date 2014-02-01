#include <dviz_core/base_movement_controller.h>

namespace demonstration_visualizer
{

const double BaseMovementController::DEFAULT_LINEAR_SPEED = 0.05;
const double BaseMovementController::DEFAULT_ANGULAR_SPEED = 0.05;

BaseMovementController::BaseMovementController()
  : last_state_(INITIAL), frames_(0),
    linear_speed_(DEFAULT_LINEAR_SPEED),
    angular_speed_(DEFAULT_ANGULAR_SPEED),
    print_transitions_(false)
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
  double goal_angle    = tf::getYaw(goal_pose.orientation);

  // Based on the previous state and the current information, decide which state to 
  // transition to.
  switch(last_state_)
  {
  case READY:
  {
    if(distance < 1.0)
      return translateToCloseGoalPosition(distance, angle_to_goal, current_angle);
  }
  case ROTATE_TO_POSITION:
  {
    if(!anglesEqual(current_angle, angle_to_goal, 0.1) && distance >= 0.1)
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
      if(anglesEqual(current_angle, angle_to_goal, 0.2))
	return translateToGoalPosition(distance);
      else
	return rotateToGoalPosition(current_angle, angle_to_goal);
    }
  }
  case TRANSLATE_TO_CLOSE_POSITION:
  {
    if(distance < /*0.1*/ 0.05)
      return rotateToGoalOrientation(current_pose.orientation, goal_pose.orientation);
    else if(distance < 1.0)
      return translateToCloseGoalPosition(distance, angle_to_goal, current_angle);
    else
      return rotateToGoalPosition(current_angle, angle_to_goal);
  }
  case ROTATE_TO_ORIENTATION:
  {
    if(anglesEqual(current_angle, goal_angle, 0.1))
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

geometry_msgs::Twist BaseMovementController::rotate(double current_angle, 
						    double goal_angle,
						    double angular_speed)
{
  bool clockwise = shortestRotationDirection(current_angle, goal_angle);

  geometry_msgs::Twist vel;
  vel.linear.x = vel.linear.y = vel.linear.z = 0;

  if(clockwise)
    vel.angular.z = -1.0 * angular_speed;
  else
    vel.angular.z = angular_speed;

  double dist = 
    angles::normalize_angle_positive(angles::normalize_angle_positive(goal_angle) - 
				      angles::normalize_angle_positive(current_angle));
  //ROS_INFO("Rotation dist = %f, ang_vel_z = %f", dist, vel.angular.z);

  if(dist == 0) return done();

  if(dist > 0 && dist <= std::abs(vel.angular.z))
  {
    //ROS_INFO("dist = %f, ||1/f * angular_z|| = %f", dist, std::abs(vel.angular.z)); 
    //ROS_INFO("decrease in angular speed: dist = %f", dist);
    if(clockwise)
    {
      //ROS_INFO("setting angular speed to %f (previously %f)", -1.0 * dist/* * getFrameRate()*/, vel.angular.z);
      vel.angular.z = -1.0 * dist; //std::floor(-1.0 * dist * getFrameRate() * 1000.0)/1000.0;
      //ROS_INFO("after rounding: %f", vel.angular.z);
    }
    else
    {
      //ROS_INFO("setting angular speed to %f (previously %f)", dist/* * getFrameRate()*/, vel.angular.z);
      vel.angular.z = dist; //std::floor(dist * getFrameRate() * 1000.0)/1000.0;
      //ROS_INFO("after rounding: %f", vel.angular.z);
    }
  }

  return vel;
}

bool BaseMovementController::anglesEqual(double current_angle,
					 double goal_angle,
					 double tolerance)
{
  bool clockwise = shortestRotationDirection(current_angle, goal_angle);
  double distance = 0.0;

  if(goal_angle > current_angle)
  {
    distance = (clockwise ? (2*M_PI - goal_angle + current_angle)
		: std::abs(goal_angle - current_angle));
  }
  else
  {
    distance = (clockwise ? (2*M_PI - current_angle + goal_angle) 
		: std::abs(current_angle - goal_angle));
  }

  return (distance <= tolerance);
}

bool BaseMovementController::shortestRotationDirection(double current_angle, 
						       double goal_angle)
{
  double A = 0.0;
  double B = 0.0;
  bool goal_angle_larger = goal_angle > current_angle;
  if(goal_angle_larger)
  {
    B = goal_angle;
    A = current_angle;
  }
  else
  {
    B = current_angle;
    A = goal_angle;
  }

  return (!goal_angle_larger && (B - A) < (2*M_PI - B + A)) ||
    (goal_angle_larger && (B - A) > (2*M_PI - B + A));  
}

void BaseMovementController::enablePrinting()
{
  print_transitions_ = true;
}

void BaseMovementController::disablePrinting()
{
  print_transitions_ = false;
}

geometry_msgs::Twist BaseMovementController::rotateToGoalPosition(double current_angle,
								  double goal_angle)
{
  printStateTransition(ROTATE_TO_POSITION);

  geometry_msgs::Twist vel = rotate(current_angle, goal_angle, angular_speed_);

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

geometry_msgs::Twist BaseMovementController::translateToCloseGoalPosition(double distance,
									  double angle_to_goal,
									  double current_angle)
{
  printStateTransition(TRANSLATE_TO_CLOSE_POSITION);

  geometry_msgs::Twist vel;

  // Just translate, no rotational motion.
  vel.linear.x = linear_speed_ * std::cos(angle_to_goal - current_angle);
  vel.linear.y = linear_speed_ * std::sin(angle_to_goal - current_angle);
  vel.angular.z = 0;

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

  geometry_msgs::Twist vel = rotate(current_angle, goal_angle, angular_speed_);

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
    if(print_transitions_)
    {
      ROS_INFO("Transitioning to state %s after %d frames in state %s.",
	       STATE_NAMES[next_state],
	       frames_,
	       STATE_NAMES[last_state_]);
    }
    last_state_ = next_state;
    frames_ = 0;
  }
}

} // namespace demonstration_visualizer
