#include<demonstration_visualizer/object_manager.h>

using namespace std;
using namespace visualization_msgs;
using namespace geometry_msgs;

ObjectManager::ObjectManager(/*CollisionChecker* c*/){
  //collision_checker_ = c;
}

void ObjectManager::addObject(Object o){
  objects_.insert(make_pair<int, Object>(o.mesh_marker_.id, o));
}

void ObjectManager::removeObject(int id) {
  objects_.erase(id);
}

void ObjectManager::clearObjects(){
  objects_.clear();
}

vector<Marker> ObjectManager::getMarkers(){
  vector<Marker> markers;
  map<int, Object>::iterator it;
  for(it = objects_.begin(); it != objects_.end(); ++it)
  {
    markers.push_back((it->second).mesh_marker_);
  }

  return markers;
}

Marker ObjectManager::getMarker(int id){
  return objects_[id].mesh_marker_;
}

bool ObjectManager::checkRobotMove(vector<double> rangles, vector<double> langles, BodyPose bp, int skip_id){
  /*
  //check robot against world
  if(!collision_checker_->checkRobotAgainstWorld(rangles, langles, bp))
    return false;
  //check robot against itself
  if(!collision_checker_->checkRobotAgainstRobot(rangles, langles, bp))
    return false;
  //check robot against all objects
  for(unsigned int i=0; i<objects_.size(); i++){
    if(i==skip_id)
      continue;
    if(!collision_checker_->checkRobotAgainstGroup(rangles, langles, bp, objects_[i].group_))
      return false;
  }
  */
  return true;
}

bool ObjectManager::checkObjectMove(int id, Pose p,
                                    vector<double> rangles, vector<double> langles, BodyPose bp){
  //if this is a static object (one that isn't supposed to move) we can put it anywhere
  if(!objects_[id].movable)
    return true;

  //collision check
  Object temp = objects_[id];
  temp.setPose(p);
  /*
  //check the object against the environment
  if(!collision_checker_->checkGroupAgainstWorld(temp.group_))
    return false;
  //check object against robot
  if(!collision_checker_->checkRobotAgainstGroup(rangles, langles, bp, temp.group_))
    return false;
  //check object against all other objects
  for(unsigned int i=0; i<objects_.size(); i++){
    if(i==id)
      continue;
    if(!collision_checker_->checkGroupAgainstGroup(objects_[i].group_,temp.group_))
      return false;
  }
  */

  //check any extera constraints
  return objects_[id].checkConstraints(p);
}

void ObjectManager::moveObject(int id, Pose p){
  objects_[id].setPose(p);
}

void ObjectManager::scaleObject(int id, double x, double y, double z) {
  objects_[id].mesh_marker_.scale.x = x;
  objects_[id].mesh_marker_.scale.y = y;
  objects_[id].mesh_marker_.scale.z = z;
}

int ObjectManager::getNumObjects() const {
  return objects_.size();
}
