/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef PR2_SIMULATOR_H
#define PR2_SIMULATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_datatypes.h>
#include <pviz/pviz.h>
#include <interactive_markers/interactive_marker_server.h>
//#include <dviz_core/base_movement_controller.h>
#include <dviz_core/base_carrot_controller.h>
#include <dviz_core/end_effector_controller.h>
#include <dviz_core/motion_recorder.h>
#include <dviz_core/object_manager.h>
#include <dviz_core/common.h>
#include <sbpl_manipulation_components_pr2/pr2_kdl_robot_model.h>
#include <tf/transform_broadcaster.h>
#include <nav_core/base_global_planner.h>

#include <Qt>
#include <QEvent>

#include <vector>
#include <string>
#include <cmath>
#include <map>

namespace demonstration_visualizer 
{

class PR2Simulator
{
public:
  enum Arm
  {
    RIGHT_ARM = 0,
    LEFT_ARM,
    NUM_ARMS
  };

  PR2Simulator(MotionRecorder *recorder, 
	       PViz *pviz, 
	       interactive_markers::InteractiveMarkerServer *int_marker_server,
	       ObjectManager* object_manager,
               int user_id = 0);

  ~PR2Simulator();

  /**
   * @brief Play the simulator.
   */
  void play();
  
  /**
   * @brief Pause the simulator now.
   */
  void pause();

  /**
   * @brief Pause the simulator later, after all snap motions have been 
   *        completed. This is a safer way to pause the simulator if we 
   *        do not want to cut short any snap motions.
   */
  void pauseLater();

  bool isPlaying() const;

  void run();
  
  /**
   * @brief Moves the robot (base and end-effectors) by first proposing new base and
   *        end-effector poses, checking these for validity (i.e. no collisions and 
   *        valid IK).
   */
  void moveRobot();

  /**
   * @brief Checks the validity of the robot's pose.
   */
  bool isValidRobotPose();

  /**
   * @brief Update the (linear/angular) velocity of the base.
   * @param[in] 
   */
  void updateBaseVelocity(const geometry_msgs::Twist &);

  /**
   * @brief Visualizes the robot using PViz.
   */
  void visualizeRobot();

  /**
   * @brief Updates the pose of the end-effector in Cartesian space
   *        based on the current joint angles, using forward kinematics.
   */
  void updateEndEffectorPose(Arm arm = RIGHT_ARM);

  bool setEndEffectorGoalPose(const geometry_msgs::Pose &pose, Arm arm = RIGHT_ARM);

  void updateEndEffectorVelocity(const geometry_msgs::Twist &twist, Arm arm = RIGHT_ARM);

  void baseMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void gripperMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void upperArmMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  /**
   * @brief Sets the linear and angular speeds of the robot in meters/frame.
   */
  void setBaseSpeed(double linear, double angular);

  /**
   * @brief Sets the speed of the end-effector in meters/frame.
   */
  void setEndEffectorSpeed(double speed, Arm arm = RIGHT_ARM);

  void resetRobot();

  void resetRobotTo(const geometry_msgs::Pose &pose, double torso_position = 0.3);

  void setRobotPose(const geometry_msgs::Pose &pose);

  void setJointStates(const sensor_msgs::JointState &joints);

  /**
   * @brief Generates and executes an interpolated trajectory for the end-
   *        effector between its current pose and the specified goal pose.
   */
  bool snapEndEffectorTo(const geometry_msgs::Pose &pose,
                         double gripper_joint = EndEffectorController::GRIPPER_OPEN_ANGLE,
                         bool snap_attached_object = true,
                         bool interpolate_position = true,
                         bool interpolate_orientation = true,
                         bool stop_while_snapping = true,
                         bool check_for_collisions = true,
                         int skip_object_id = -1,
                         Arm arm = RIGHT_ARM);

  bool isSnapDone(Arm arm = RIGHT_ARM) const;

  /**
   * @brief Provides a way to notify the simulator of key events recieved
   *        elsewhere
   */
  void processKeyEvent(int key, int type);

  void updateEndEffectorMarker(Arm arm = RIGHT_ARM);

  void updateUpperArmMarker(Arm arm = RIGHT_ARM);

  void updateEndEffectorMarkerVelocity(const geometry_msgs::Twist &twist, Arm arm = RIGHT_ARM);

  bool isBaseMoving() const;

  /**
   * @brief Checks to see if the given pose is a valid pose for the end effector
   *        using FK with the current joint angles
   */
  bool isValidEndEffectorPose(const geometry_msgs::Pose &pose, Arm arm = RIGHT_ARM);

  bool validityCheck(const std::vector<double>& rangles, 
                     const std::vector<double>& langles, 
                     BodyPose& bp, 
                     const geometry_msgs::Pose& obj);

  void computeObjectPose(const std::vector<double> &eef, BodyPose bp, geometry_msgs::Pose& obj, Arm arm = RIGHT_ARM);

  void attach(int id, KDL::Frame transform, Arm arm = RIGHT_ARM);

  void detach(Arm arm = RIGHT_ARM);

  /**
   * @brief Returns true if an object is attached to the end-effector of the robot
   */
  bool isObjectAttached(Arm arm = RIGHT_ARM) const;

  /**
   * @brief If an object is attached to the (right) end-effector, this will
   *        return the object_in_gripper transform (i.e. the gripper to 
   *        object transform.)
   */
  KDL::Frame getAttachedTransform(Arm arm = RIGHT_ARM) const;

  void showEndEffectorWorkspaceArc();

  geometry_msgs::Pose getBasePose() const;

  geometry_msgs::Pose getEndEffectorPose(Arm arm = RIGHT_ARM);

  geometry_msgs::Pose getEndEffectorPoseInBase(Arm arm = RIGHT_ARM) const;

  geometry_msgs::Pose getEndEffectorMarkerPose(Arm arm = RIGHT_ARM);

  bool getObjectPose(geometry_msgs::Pose &object_pose, Arm arm = RIGHT_ARM);

  void setMoveEndEffectorWhileDragging(bool move);

  /**
   * @brief Performs a basic search to look for the closest valid IK solution for various 
   *        (x, y, z) positions within a given range of the current position
   *
   */
  bool closestValidEndEffectorPosition(const geometry_msgs::Pose &current_pose,
				       const std::vector<std::pair<double, double> > &intervals,
				       const std::vector<double> &d,
				       double &x, double &y, double &z,
                                       Arm arm,
				       bool verbose = false);

  bool canMoveRobotMarkers() const;

  void setMoveRobotMarkers(bool move);

  void enableOrientationControl(Arm arm = RIGHT_ARM);
  
  void disableOrientationControl(Arm arm = RIGHT_ARM);

  void enableUpperArmRollControl(Arm arm = RIGHT_ARM);

  void disableUpperArmRollControl(Arm arm = RIGHT_ARM);

  double getTorsoPosition() const;

  void setTorsoPosition(double position);

  void setIgnoreCollisions(bool ignore);
  
  void setFrameRate(double rate);

  double getFrameRate() const;

  bool isBasePlanDone() const;

private:
  void unnormalize(std::vector<sensor_msgs::JointState> &joints);

  bool playing_;
  int user_id_;
  double frame_rate_;

  bool move_end_effector_while_dragging_;

  void updateTransforms();

  bool attached_object_[NUM_ARMS];
  int attached_id_[NUM_ARMS];
  KDL::Frame attached_transform_[NUM_ARMS];

  ObjectManager* object_manager_;
  PViz *pviz_;
  interactive_markers::InteractiveMarkerServer *int_marker_server_;
  std::vector<EndEffectorController> end_effector_controller_;

  nav_core::BaseGlobalPlanner *base_planner_;
  std::vector<geometry_msgs::PoseStamped> base_plan_;
  int base_plan_index_;

  MotionRecorder *recorder_;

  ros::Subscriber vel_cmd_sub_;
  ros::Subscriber end_effector_vel_cmd_sub_[NUM_ARMS];
  
  ros::Publisher marker_pub_;

  geometry_msgs::Twist vel_cmd_;
  geometry_msgs::Twist key_vel_cmd_;
  geometry_msgs::Twist end_effector_vel_cmd_[NUM_ARMS];
  geometry_msgs::Twist end_effector_marker_vel_[NUM_ARMS];
  geometry_msgs::PoseStamped base_pose_;
  geometry_msgs::PoseStamped end_effector_pose_[NUM_ARMS];
  geometry_msgs::PoseStamped end_effector_goal_pose_[NUM_ARMS];
  geometry_msgs::Pose upper_arm_roll_pose_[NUM_ARMS];
  sensor_msgs::JointState joint_states_;
  double torso_position_;

  visualization_msgs::MarkerArray robot_markers_;

  geometry_msgs::PoseStamped goal_pose_;

  sbpl_arm_planner::PR2KDLRobotModel kdl_robot_model_[NUM_ARMS];
  KDL::Frame map_in_torso_lift_link_;
  std::map<std::string, int> joints_map_;

  tf::TransformBroadcaster tf_broadcaster_;

  std::vector<sensor_msgs::JointState> snap_motion_[NUM_ARMS];
  int snap_motion_count_[NUM_ARMS];
  bool snap_object_[NUM_ARMS];
  bool stop_while_snapping_;

  bool moving_gripper_marker_;
  bool move_robot_markers_;

  bool pause_requested_;

  bool goal_orientation_changed_[NUM_ARMS];
  geometry_msgs::Quaternion end_effector_goal_orientation_[NUM_ARMS];

  double delta_arm_roll_[NUM_ARMS];

  bool ignore_collisions_;

};

} // namespace demonstration_visualizer

#endif // PR2_SIMULATOR_H
