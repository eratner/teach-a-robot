#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

#include<demonstration_visualizer/object.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Pose.h>
#include<pviz/pviz.h>
#include<pr2_collision_checker/pr2_collision_space.h>
#include <ros/package.h>

#include <map>

class ObjectManager{
  public:
    ObjectManager(std::string rarm_file, std::string larm_file);
    void initializeCollisionChecker(std::vector<double> dims, std::vector<double> origin);
    void addObject(Object o);
    
    bool addObjectFromFile(visualization_msgs::Marker &mesh_marker,
			   const std::string &collision_model_file, 
			   bool movable = false);

    void removeObject(int id);
    void clearObjects();
    std::vector<visualization_msgs::Marker> getMovedMarkers();
    std::vector<visualization_msgs::Marker> getMarkers();
    visualization_msgs::Marker getMarker(int id);
    bool checkRobotMove(std::vector<double> rangles, std::vector<double> langles, BodyPose bp, int skip_id);
    bool checkObjectMove(int id, geometry_msgs::Pose p,
                         std::vector<double> rangles, std::vector<double> langles, BodyPose bp);
    void moveObject(int id, geometry_msgs::Pose p);

    void scaleObject(int id, double x, double y, double z);

    int getNumObjects() const;

    std::vector<Object> getObjects() const;

    bool addObjectsFromOccupiedVoxelsFile(std::string filename);

    bool writeObjectsToOccupiedVoxelsFile(std::string filename);

  private:
    pr2_collision_checker::PR2CollisionSpace* collision_checker_;
    std::map<int, Object> objects_;
    std::string rarm_file_;
    std::string larm_file_;
    bool debug_visualizations_;
    bool load_objects_from_voxels_file_;
};

#endif
