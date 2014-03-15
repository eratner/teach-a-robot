/**
 * @brief A finite-state automata (FSA) based carrot-controller for
 *        generating a list of poses for the base of the robot given
 *        input start and goal poses
 * @author Ellis Ratner
 * @date March 2014
 */
#ifndef BASE_CARROT_CONTROLLER_H
#define BASE_CARROT_CONTROLLER_H

#include <ros/ros.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>
#include <cmath>

namespace demonstration_visualizer
{

static const char *STATE_NAMES[] = { "INITIAL",
                                     "READY",
				     "ROTATE_TO_POSITION",
				     "TRANSLATE_TO_POSITION",
				     "TRANSLATE_TO_CLOSE_POSITION",
				     "ROTATE_TO_ORIENTATION",
				     "DONE" };

class BaseCarrotController : public nav_core::BaseGlobalPlanner
{
public:
  static const double DEFAULT_LINEAR_SPEED;
  static const double DEFAULT_ANGULAR_SPEED;

  enum State { INITIAL = 0,
	       READY,
               ROTATE_TO_POSITION,
	       TRANSLATE_TO_POSITION,
	       TRANSLATE_TO_CLOSE_POSITION,
	       ROTATE_TO_ORIENTATION,
	       DONE };

  BaseCarrotController();

  virtual ~BaseCarrotController();

  void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

  bool makePlan(const geometry_msgs::PoseStamped &start,
		const geometry_msgs::PoseStamped &goal,
		std::vector<geometry_msgs::PoseStamped> &plan);

  geometry_msgs::Twist getNextVelocities(const geometry_msgs::PoseStamped &start,
					 const geometry_msgs::PoseStamped &goal);

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
  geometry_msgs::Twist rotate(double current_angle, double goal_angle, double angular_speed);

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

  void enablePrinting();

  void disablePrinting();

private:
  State last_state_;
  int frames_;
  double linear_speed_;
  double angular_speed_;

  bool print_transitions_;
  
  // States
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

#endif // BASE_CARROT_CONTROLLER_H
