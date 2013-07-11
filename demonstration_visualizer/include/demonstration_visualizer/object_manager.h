#ifndef OBJECT_MANAGER_H
#define OBJECT_MANAGER_H

#include<demonstration_visualizer/object.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Pose.h>
#include<pviz/pviz.h>
//#include<collision_checker.h>

#include <map>

class ObjectManager{
  public:
    ObjectManager(/*CollisionChecker* c*/);
    void addObject(Object o);
    void removeObject(int id);
    void clearObjects();
    std::vector<visualization_msgs::Marker> getMarkers();
    visualization_msgs::Marker getMarker(int id);
    bool checkRobotMove(std::vector<double> rangles, std::vector<double> langles, BodyPose bp, int skip_id);
    bool checkObjectMove(int id, geometry_msgs::Pose p,
                         std::vector<double> rangles, std::vector<double> langles, BodyPose bp);
    void moveObject(int id, geometry_msgs::Pose p);

    void scaleObject(int id, double x, double y, double z);

    int getNumObjects() const;

  private:
    //CollisionChecker* collision_checker_;
    std::map<int, Object> objects_;
};

#endif
