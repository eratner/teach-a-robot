#include<demonstration_visualizer/object.h>
#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Pose.h>
#include<vector>
//#include<collision_checker.h>

class ObjectManager{
  public:
    ObjectManager(CollisionChecker* c);
    void addObject(Object o);
    void clearObjects();
    std::vector<visualization_msgs::Marker> getMarkers();
    visualization_msgs::Marker getMarker(int id);
    bool checkMove(int id, geometry_msgs::Pose p,
                   std::vector<double> rangles, std::vector<double> langles, BodyPose bp);
    void moveObject(int id, geometry_msgs::Pose p);

  private:
    CollisionChecker* collision_checker_;
    vector<Object> objects_;
};

