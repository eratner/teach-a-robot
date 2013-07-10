/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef DEMONSTRATION_SCENE_MANAGER_H
#define DEMONSTRATION_SCENE_MANAGER_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <demonstration_visualizer/visualization_helpers.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tinyxml.h>
#include <string>
#include <vector>
#include <cmath>

namespace demonstration_visualizer {

/**
 * @brief This manages the demonstration scene and current task. Demonstration
 *        scenes are comprised of a collection of meshes, and tasks are 
 *        essentially ordered lists of goals. Both scenes and tasks can be 
 *        saved to/loaded from scene and task XML files, respectively.
 */
class DemonstrationSceneManager
{
public:
  struct GraspableGoal
  {
    int number_;
    std::string description_;
    // @todo in the future, the goal marker will be encapsulated in 
    // an object class.
    visualization_msgs::Marker marker_;
    geometry_msgs::Pose gripper_pose_;
  };

  DemonstrationSceneManager();

  ~DemonstrationSceneManager();

  /**
   * @brief Updates the meshes and goal markers according to the current 
   *        state of the demonstration scene.
   */
  void updateScene();

  /**
   * @brief Loads a scene file (.xml format), and displays it, overwriting 
   *        any existing scene that has been displayed.
   * @return The max mesh id of the meshes in the loaded scene.
   */
  int loadScene(const std::string &filename);

  void saveScene(const std::string &filename);

  void loadTask(const std::string &filename);

  void saveTask(const std::string &filename);

  void addMeshFromFile(const std::string &filename, int mesh_id);

  void addMesh(const visualization_msgs::Marker &marker, bool attach_interactive_marker = false);

  bool visualizeMesh(int mesh_id, bool attach_interactive_marker);

  visualization_msgs::Marker getMesh(int mesh_id);
  
  bool updateMeshPose(int mesh_id, const geometry_msgs::Pose &pose);

  bool updateMeshScale(int mesh_id, double x, double y, double z);

  bool removeMesh(int mesh_id);

  // Adds an additional goal to the current task, at the end. 
  void addGoal(const geometry_msgs::Pose &pose = geometry_msgs::Pose(), 
	       const std::string &desc = "",
	       const std::string &frame = "/map");

  bool moveGoal(int goal_number, const geometry_msgs::Pose &pose);

  GraspableGoal getGoal(int goal_number);

  /**
   * @brief Determines whether the given position is at the position of the given goal, 
   *        within a tolerance (in meters).
   */
  bool hasReachedGoal(int goal_number, const geometry_msgs::Pose &pose, double tolerance = 0.1);

  std::vector<visualization_msgs::Marker> getMeshes() const;

  std::vector<GraspableGoal> getGoals() const;

  int getNumGoals() const;

  bool goalsChanged() const;

  void setGoalsChanged(bool changed = true);

  void setGoalDescription(int goal_number, const std::string &desc);

  std::string getGoalDescription(int goal_number) const;

  void setGripperPose(int goal_number, const geometry_msgs::Pose &gripper_pose);

  geometry_msgs::Pose getGripperPose(int goal_number);

  int getCurrentGoal() const;

  void setCurrentGoal(int);

  bool editGoalsMode() const;
  
  void setEditGoalsMode(bool);

  bool editMeshesMode() const;

  void setEditMeshesMode(bool);

  void setMeshesChanged(bool changed = true);

  bool meshesChanged() const;

  bool taskDone() const;

private:
  bool goals_changed_;
  bool meshes_changed_;
  bool edit_goals_mode_;
  bool edit_meshes_mode_;
  int current_goal_;

  /**
   * @brief Returns the iterator at the position of the marker in the given vector
   *        of markers with the specified marker id, or end() if it cannot find 
   *        such a marker.
   */
  std::vector<visualization_msgs::Marker>::iterator findMarker(std::vector<visualization_msgs::Marker> &,
							       int);

  void processGoalFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void processMeshFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  std::vector<visualization_msgs::Marker> meshes_;
  //std::vector<visualization_msgs::Marker> goals_;
  std::vector<GraspableGoal> goals_;
  //std::vector<std::string> goal_descriptions_;

  interactive_markers::InteractiveMarkerServer int_marker_server_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback goal_feedback_;
  interactive_markers::InteractiveMarkerServer::FeedbackCallback mesh_feedback_;

  ros::Publisher marker_pub_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_SCENE_MANAGER_H
