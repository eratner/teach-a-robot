/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef DEMONSTRATION_SCENE_MANAGER_H
#define DEMONSTRATION_SCENE_MANAGER_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <dviz_core/visualization_helpers.h>
#include <dviz_core/common.h>
#include <dviz_core/object_manager.h>
#include <dviz_core/goal.h>
#include <pviz/pviz.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tinyxml.h>
#include <string>
#include <vector>
#include <cmath>
#include <boost/filesystem.hpp>

namespace demonstration_visualizer
{

/**
 * @brief This manages the demonstration scene and current task. Demonstration
 *        scenes are comprised of a collection of meshes, and tasks are 
 *        essentially ordered lists of goals. Both scenes and tasks can be 
 *        saved to/loaded from scene and task XML files, respectively
 */
class DemonstrationSceneManager
{
public:
  static const std::string GOAL_MARKER_NAMESPACE;

  DemonstrationSceneManager(PViz *pviz, interactive_markers::InteractiveMarkerServer *int_marker_server, 
			    ObjectManager* object_manager, int user_id = 0);

  ~DemonstrationSceneManager();

  /**
   * @brief Updates the meshes and goal markers according to the current 
   *        state of the demonstration scene
   */
  void updateScene();

  /**
   * @brief Loads a scene file (.xml format), and displays it, overwriting 
   *        any existing scene that has been displayed
   * @return The max mesh id of the meshes in the loaded scene
   */
  int loadScene(const std::string &filename);

  void saveScene(const std::string &filename);

  bool loadTask(const std::string &filename, bool randomize = false);

  void saveTask(const std::string &filename);

  void resetTask();

  /**
   * @brief Randomizes any currently loaded pick-and-place tasks (so the assumption
   *        is that pairs of goals must remain in the same order, e.g. 0 & 1, 2 & 3, etc.)
   */
  void randomizePickAndPlaceTask();

  void addMeshFromFile(const std::string &filename, int mesh_id, 
		       const std::string &label = "", bool movable = false,
                       bool attach_interactive_marker = true);

  void addMesh(const visualization_msgs::Marker &marker,
  	       bool attach_interactive_marker = false,
	       const std::string &label = "",
  	       bool movable = false);

  void visualizeMesh(const visualization_msgs::Marker &marker, bool attach_interactive_marker);

  bool updateMeshPose(int mesh_id, const geometry_msgs::Pose &pose);

  void updateMeshScale(int mesh_id, double x, double y, double z);

  void removeMesh(int mesh_id);

  /**
   * @brief Adds an additional goal to the current task, at the end
   */
  void addGoal(const std::string &desc = "",
	       Goal::GoalType type = Goal::PICK_UP,
	       int object_id = 0,
               bool ignore_yaw = false);

  bool moveGoal(int goal_number, const geometry_msgs::Pose &pose);

  Goal *getGoal(int goal_number);

  std::vector<visualization_msgs::Marker> getMeshes() const;

  visualization_msgs::Marker getMeshMarker(int mesh_id) const;

  std::vector<Object> getObjects() const;

  std::vector<Goal *> getGoals() const;

  int getNumGoals() const;

  bool goalsChanged() const;

  void setGoalsChanged(bool changed = true);

  void setGoalDescription(int goal_number, const std::string &desc);

  std::string getGoalDescription(int goal_number) const;

  bool setGraspPose(int goal_number, const geometry_msgs::Pose &grasp);

  /**
   * @return the pose of the grasp in the map frame
   */
  geometry_msgs::Pose getGraspPose(int goal_number);

  /**
   * @return the pose of the grasp in the objects frame
   */
  geometry_msgs::Pose getGraspPoseObjectFrame(int goal_number);

  geometry_msgs::Pose getObjectPose(int object_id);

  geometry_msgs::Pose getCurrentGoalPose();

  int getCurrentGoal() const;

  void setCurrentGoal(int);

  bool editGoalsMode() const;
  
  void setEditGoalsMode(bool);

  bool editMeshesMode() const;

  void setEditMeshesMode(bool);

  bool taskDone() const;

  geometry_msgs::Pose getInitialRobotPose() const;

  double getInitialTorsoPosition() const;

  std::string getTaskName() const;

  void setTaskName(const std::string &);

private:
  int user_id_;

  bool goals_changed_;
  bool edit_goals_mode_;
  bool edit_meshes_mode_;
  int current_goal_;

  std::string task_name_;

  geometry_msgs::Pose initial_robot_pose_;
  double initial_torso_position_;

  ObjectManager *object_manager_;

  void processGoalFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void processMeshFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void drawGoal(Goal *goal, bool attach_interactive_marker = false);

  void hideGoal(Goal *goal);

  std::vector<Goal *> goals_;

  interactive_markers::InteractiveMarkerServer *int_marker_server_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback goal_feedback_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback mesh_feedback_;

  PViz *pviz_;

  ros::Publisher marker_pub_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_SCENE_MANAGER_H
