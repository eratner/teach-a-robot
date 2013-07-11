#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

#include<demonstration_visualizer/object.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Pose.h>
#include<vector>
#include<pviz/pviz.h>
//#include<collision_checker.h>

class ObjectManager{
  public:
    ObjectManager(/*CollisionChecker* c*/);
    void addObject(Object o);
    void clearObjects();
    std::vector<visualization_msgs::Marker> getMarkers();
    visualization_msgs::Marker getMarker(int id);
    bool checkRobotMove(std::vector<double> rangles, std::vector<double> langles, BodyPose bp, int skip_id);
    bool checkObjectMove(int id, geometry_msgs::Pose p,
                         std::vector<double> rangles, std::vector<double> langles, BodyPose bp);
    void moveObject(int id, geometry_msgs::Pose p);

  private:
    //CollisionChecker* collision_checker_;
    std::vector<Object> objects_;
};

#endif
