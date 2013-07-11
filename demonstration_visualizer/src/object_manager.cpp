#include<demonstration_visualizer/object_manager.h>

using namespace std;
using namespace visualization_msgs;
using namespace geometry_msgs;

ObjectManager::ObjectManager(){
  //PR2CollisionChecker* collision_checker_;
}

void ObjectManager::addObject(Object o){
  objects_.push_back(o);

  if(!objects_.back().movable){
    //collision_checker_->addCollisionObject(...)
  }
}

void ObjectManager::clearObjects(){
  objects_.clear();
}

vector<Marker> ObjectManager::getMarkers(){
  vector<Marker> markers;
  for(unsigned int i=0; i<objects_.size(); i++)
    markers.push_back(objects_[i].mesh_marker_);
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

