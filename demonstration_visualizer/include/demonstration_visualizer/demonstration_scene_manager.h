#ifndef DEMONSTRATION_SCENE_MANAGER_H
#define DEMONSTRATION_SCENE_MANAGER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tinyxml.h>
#include <string>
#include <vector>
#include <cmath>

class DemonstrationSceneManager
{
public:
  DemonstrationSceneManager();

  ~DemonstrationSceneManager();

  void loadScene(const std::string &filename);

  void saveScene(const std::string &filename);

  void loadTask(const std::string &filename);

  void saveTask(const std::string &filename);

  void addMesh(const visualization_msgs::Marker &marker);
  
  bool updateMeshPose(int mesh_id, const geometry_msgs::Pose &pose);

  bool removeMesh(int mesh_id);

  // Adds an additional goal to the current task, at the end. 
  void addGoal(const geometry_msgs::Pose &pose = geometry_msgs::Pose(), 
	       const std::string &desc = "",
	       const std::string &frame = "/map");

  bool moveGoal(int goal_number, const geometry_msgs::Pose &pose);

  visualization_msgs::Marker getGoal(int goal_number);

  bool hasReachedGoal(int goal_number, const geometry_msgs::Pose &pose);

  std::vector<visualization_msgs::Marker> getMeshes() const;

  std::vector<visualization_msgs::Marker> getGoals() const;

  int getNumGoals() const;

  bool goalsChanged() const;

  void setGoalsChanged(bool);

  void setGoalDescription(int goal_number, const std::string &desc);

  std::string getGoalDescription(int goal_number) const;

private:
  bool goals_changed_;

  std::vector<visualization_msgs::Marker> meshes_;
  std::vector<visualization_msgs::Marker> goals_;
  std::vector<std::string> goal_descriptions_;

};

#endif // DEMONSTRATION_SCENE_MANAGER_H
