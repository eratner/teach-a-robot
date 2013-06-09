#ifndef DEMONSTRATION_SCENE_MANAGER_H
#define DEMONSTRATION_SCENE_MANAGER_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tinyxml.h>
#include <string>
#include <vector>

class DemonstrationSceneManager
{
public:
  DemonstrationSceneManager();

  ~DemonstrationSceneManager();

  void loadScene(const std::string &filename);

  void saveScene(const std::string &filename);

  void addMesh(const visualization_msgs::Marker &marker);
  
  bool updateMeshPose(int mesh_id, const geometry_msgs::Pose &pose);

  bool removeMesh(int mesh_id);

  std::vector<visualization_msgs::Marker> getMeshes() const;

private:
  std::vector<visualization_msgs::Marker> meshes_;

};

#endif // DEMONSTRATION_SCENE_MANAGER_H
