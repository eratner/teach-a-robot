/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef BASE_MOVEMENT_CONTROLLER_H
#define BASE_MOVEMENT_CONTROLLER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <cmath>

namespace demonstration_visualizer {

static const char *STATE_NAMES[] = { "INITIAL",
                                     "READY",
				     "ROTATE_TO_POSITION",
				     "TRANSLATE_TO_POSITION",
				     "TRANSLATE_TO_CLOSE_POSITION",
				     "ROTATE_TO_ORIENTATION",
				     "DONE" };

class BaseMovementController
{
public:
  enum State { INITIAL = 0,
	       READY,
               ROTATE_TO_POSITION,
	       TRANSLATE_TO_POSITION,
	       TRANSLATE_TO_CLOSE_POSITION,
	       ROTATE_TO_ORIENTATION,
	       DONE };

  BaseMovementController();

  virtual ~BaseMovementController();

  geometry_msgs::Twist getNextVelocities(const geometry_msgs::Pose &current_pose,
					 const geometry_msgs::Pose &goal_pose);

  void setState(State state);

  State getState() const;

  void setLinearSpeed(double);

  double getLinearSpeed() const;

  void setAngularSpeed(double);
  
  double getAngularSpeed() const;

  /**
   * @brief A helper method to compute the necessary angular velocity to achieve
   *        the goal angle given the current angle, both in the range [0, 2 \pi),
   *        and a set angular speed.
   */
  static geometry_msgs::Twist rotate(double current_angle, double goal_angle, double angular_speed);

  /**
   * @brief A helper method that determines whether two angles, both in the range 
   *        [0, 2 \pi), are equal within a given tolerance.
   */
  static bool anglesEqual(double current_angle, double goal_angle, double tolerance = 0.1);

  /**
   * @brief A helper method to determine the direction of rotation (clockwise or 
   *        counter-clockwise) that yields the shortest angular distance between
   *        the current and goal angles.
   * @return Returns true if the shortest direction is clockwise rotation; returns
   *         false if shortest direction is counter-clockwise.
   */
  static bool shortestRotationDirection(double current_angle, double goal_angle);

private:
  State last_state_;
  int frames_;
  double linear_speed_;
  double angular_speed_;
  
  // States.
  geometry_msgs::Twist rotateToGoalPosition(double current_angle,
					    double goal_angle);

  geometry_msgs::Twist translateToGoalPosition(double distance);

  geometry_msgs::Twist translateToCloseGoalPosition(double distance,
						    double angle_to_goal,
						    double current_angle);

  geometry_msgs::Twist rotateToGoalOrientation(const geometry_msgs::Quaternion &current_orientation,
					       const geometry_msgs::Quaternion &goal_orientation);

  geometry_msgs::Twist done();

  geometry_msgs::Twist initial();

  void printStateTransition(State next_state);

};

} // namespace demonstration_visualizer

#endif // BASE_MOVEMENT_CONTROLLER_H
