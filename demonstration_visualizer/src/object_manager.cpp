#include <demonstration_visualizer/object_manager.h>
#include <sbpl_geometry_utils/SphereEncloser.h>

using namespace std;
using namespace visualization_msgs;
using namespace geometry_msgs;

ObjectManager::ObjectManager(std::string rarm_filename, std::string larm_filename){
  rarm_file_ = rarm_filename;
  larm_file_ = larm_filename;
  enable_debug_visualizations_ = false;
  disable_collision_checking_ = false;
  load_objects_from_voxels_file_ = false;
  collision_checker_ = NULL;
   
  ros::NodeHandle ph("~");
  ph.param("enable_debug_visualizations", enable_debug_visualizations_, false);
  ph.param("disable_collision_checking",  disable_collision_checking_, false);

  ROS_DEBUG("[om] Initializing object manager.");
  ROS_DEBUG("[om] right_arm_file: %s",rarm_file_.c_str());
  ROS_DEBUG("[om]  left_arm_file: %s",larm_file_.c_str());
}

void ObjectManager::initializeCollisionChecker(vector<double> dims, vector<double> origin){
  ROS_DEBUG("[om] Initializing the collision checker.");

  if(disable_collision_checking_)
  {
    ROS_INFO("[om] Collision checking is disabled. Not initializing the collision checker.");
    return;
  }

  if(collision_checker_ != NULL)
  {
    ROS_ERROR("[om] Deleting the previous collision checker. Initializing new one.");
    delete collision_checker_;
  }

  ROS_DEBUG("[om] right_arm: %s",rarm_file_.c_str());
  ROS_DEBUG("[om] left_arm: %s",larm_file_.c_str());
  collision_checker_ = new pr2_collision_checker::PR2CollisionSpace(rarm_file_, 
                                                                    larm_file_, 
                                                                    dims, origin, 
                                                                    0.02, "/map");
  if(!collision_checker_->init())
  {
    ROS_ERROR("[om] Failed to initialize the collision checker.");
    return;
  }
  collision_checker_->visualizeResult(enable_debug_visualizations_);
  ROS_INFO("[om] Initialized the collision checker.");
}


void ObjectManager::addObject(Object o){

  objects_.insert(make_pair<int, Object>(o.mesh_marker_.id, o));

  if(disable_collision_checking_)
    return;

  if(!o.movable && !load_objects_from_voxels_file_){
    if(!collision_checker_->addCollisionObjectMesh(o.mesh_marker_.mesh_resource, o.mesh_marker_.pose, o.label))
      ROS_ERROR("[om] Failed to add static object.");
  }
}

bool ObjectManager::addObjectFromFile(visualization_msgs::Marker &mesh_marker,
				      const std::string &collision_model_file,
				      bool movable)
{
  ROS_INFO("[om] Adding object from file %s.", collision_model_file.c_str());
  TiXmlDocument doc(collision_model_file.c_str());
  if(!doc.LoadFile())
  {
    ROS_ERROR("[om] Failed to load file %s!", collision_model_file.c_str());
    return false;
  }

  TiXmlHandle doc_handle(&doc);
  TiXmlElement *element;
  TiXmlHandle root_handle(0);

  element = doc_handle.FirstChildElement().Element();
  if(!element)
  {
    // @todo error
    return false;
  }

  root_handle = TiXmlHandle(element);

  ROS_INFO("[om] Reading %s...", element->Value());

  // Read the mesh information.
  element = root_handle.FirstChild("object").Element();
  if(element == NULL)
  {
    ROS_ERROR("[om] Element is null while trying to parse collision_model file.");
    return false;
  }

  std::string label;
  if(element->QueryStringAttribute("label", &label) != TIXML_SUCCESS)
  {
    ROS_ERROR("[om] label for the collision_model is missing.");
    return false;
  }

  if(element->QueryStringAttribute("mesh_resource", &mesh_marker.mesh_resource) != TIXML_SUCCESS)
  {
    ROS_ERROR("[om] mesh_resource for the collision_model is missing.");
    return false;
  }

  Object o(mesh_marker);
  o.label = label;
  o.movable = movable;
  geometry_msgs::Pose pose = mesh_marker.pose;
  o.group_.name = label;
  o.group_.f.p[0] = pose.position.x;
  o.group_.f.p[1] = pose.position.y;
  o.group_.f.p[2] = pose.position.z;
  o.group_.f.M = KDL::Rotation::Quaternion(pose.orientation.x,
                                           pose.orientation.y,
                                           pose.orientation.z,
                                           pose.orientation.w);

  if(movable)
  {
    ROS_DEBUG("[om] movable! Reading the spheres...");
    element = element->NextSiblingElement("sphere");

    if(!element)
    {
      // If no spheres found in file, generate them
      double enclosing_sphere_radius = 0.04;
      ROS_INFO("[om] No spheres were found for '%s', so generating them instead with radius, %0.3fm", label.c_str() , enclosing_sphere_radius); 
      std::vector<int> triangles;
      std::vector<geometry_msgs::Point> vertices; 
      if(!leatherman::getMeshComponentsFromResource(mesh_marker.mesh_resource, mesh_marker.scale, triangles, vertices))
      {
        ROS_ERROR("[om] Failed to get triangles & indeces from the mesh resource.");
        return false;
      }
      std::vector<std::vector<double> > sph;
      sbpl::SphereEncloser::encloseMesh(vertices, triangles, enclosing_sphere_radius, sph);

      pr2_collision_checker::Sphere s;
      s.radius = enclosing_sphere_radius;
      s.priority = 1;
      for(size_t k = 0; k < sph.size(); ++k)
      {
        s.name = boost::lexical_cast<string>(int(k));
        s.v.x(sph[k][0]);
        s.v.y(sph[k][1]);
        s.v.z(sph[k][2]);
        o.group_.spheres.push_back(s);
      }
      ROS_INFO("[om] Generated %d spheres for object group '%s'.", int(o.group_.spheres.size()), label.c_str());
    }
    else
    {
      // Read the spheres list from this file.
      for(element; element; element = element->NextSiblingElement("sphere"))
      {
        pr2_collision_checker::Sphere s;
        int id;
        if(element->QueryIntAttribute("id", &id) != TIXML_SUCCESS)
        {
          ROS_ERROR("[om] Failed to read id for sphere!");
          return false;
        }
        s.name = boost::lexical_cast<string>(id);
        double temp;
        if(element->QueryDoubleAttribute("x", &temp) != TIXML_SUCCESS)
        {
          ROS_ERROR("[om] Failed to read x-value for sphere!");
          return false;
        }
        s.v.x(temp);
        if(element->QueryDoubleAttribute("y", &temp) != TIXML_SUCCESS)
        {
          ROS_ERROR("[om] Failed to read y-value for sphere!");
          return false;
        }
        s.v.y(temp);
        if(element->QueryDoubleAttribute("z", &temp) != TIXML_SUCCESS)
        {
          ROS_ERROR("[om] Failed to read z-value for sphere!");
          return false;
        }
        s.v.z(temp);
        if(element->QueryDoubleAttribute("radius", &s.radius) != TIXML_SUCCESS)
        {
          ROS_ERROR("[om] Failed to read radius for sphere!");
          return false;
        }
        s.priority = 1;
        o.group_.spheres.push_back(s);
        ROS_DEBUG("[om] [sphere] name: %s  x: %0.3f y: %0.3f z: %0.3f radius: %0.3f", s.name.c_str(), s.v.x(), s.v.y(), s.v.z(), s.radius);
      }
      ROS_INFO("[om] label: %s   #_spheres: %d", o.group_.name.c_str(), int(o.group_.spheres.size()));
    }
    if(enable_debug_visualizations_ && !disable_collision_checking_)
      collision_checker_->visualizeGroup(o.group_, o.label+ boost::lexical_cast<string>(o.mesh_marker_.id), 0);
  }

  addObject(o);
  return true;
}

void ObjectManager::removeObject(int id) {
  objects_.erase(id);
}

void ObjectManager::clearObjects(){
  objects_.clear();
}

vector<Marker> ObjectManager::getMovedMarkers(){
  vector<Marker> markers;
  map<int, Object>::iterator it;
  for(it = objects_.begin(); it != objects_.end(); ++it){
    if(it->second.redraw){
      markers.push_back((it->second).mesh_marker_);
      it->second.redraw = false;
    }
  }
  return markers;
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
  double dist;
  if(collision_checker_ == NULL || disable_collision_checking_)
    return true;

  //check robot against world
  ROS_DEBUG("[om] Collision checking time! robot-world first");
  if(!collision_checker_->checkRobotAgainstWorld(rangles, langles, bp, false, dist))
  {
    return false;
  }
  //check robot against itself
  ROS_DEBUG("[om] Collision checking time! robot-robot first");
  if(!collision_checker_->checkRobotAgainstRobot(rangles, langles, bp, false, dist))
    return false;
  //check robot against all objects
  for(unsigned int i=0; i<objects_.size(); i++){
    if(int(i)==skip_id)
      continue;
    if(!collision_checker_->checkRobotAgainstGroup(rangles, langles, bp, &(objects_[i].group_), false/*true*/, false, dist))
      return false;
  }
  return true;
}

bool ObjectManager::checkObjectMove(int id, Pose p,
                                    vector<double> rangles, vector<double> langles, BodyPose bp){
  //if this is a static object (one that isn't supposed to move) we can put it anywhere
  if(!objects_[id].movable || disable_collision_checking_)
    return true;

  //collision check
  Object temp = objects_[id];
  temp.setPose(p);
  double dist;
  //check the object against the environment
  if(!collision_checker_->checkGroupAgainstWorld(&(temp.group_), dist))
    return false;
  //check object against robot
  if(!collision_checker_->checkRobotAgainstGroup(rangles, langles, bp, &(temp.group_), false, false, dist))
    return false;
  //check object against all other objects
  for(unsigned int i=0; i<objects_.size(); i++){
    if(int(i)==id)
      continue;
    if(!collision_checker_->checkGroupAgainstGroup(&(objects_[i].group_),&(temp.group_),dist))
      return false;
  }

  //check any extra constraints
  return objects_[id].checkConstraints(p);
}

void ObjectManager::moveObject(int id, Pose p){
  objects_[id].setPose(p);
  objects_[id].redraw = true;
}

void ObjectManager::scaleObject(int id, double x, double y, double z) {
  objects_[id].mesh_marker_.scale.x = x;
  objects_[id].mesh_marker_.scale.y = y;
  objects_[id].mesh_marker_.scale.z = z;

  if(objects_[id].movable){
    for(unsigned int i=0; i<objects_[i].group_.spheres.size(); i++){
      KDL::Vector v = objects_[i].group_.spheres[i].v;
      double radius = objects_[i].group_.spheres[i].radius;

      objects_[i].group_.spheres[i].radius = radius*x;
      objects_[i].group_.spheres[i].v.x(v.x()*x);
      objects_[i].group_.spheres[i].v.y(v.y()*y);
      objects_[i].group_.spheres[i].v.z(v.z()*z);
    }
  }
}

int ObjectManager::getNumObjects() const {
  return objects_.size();
}

vector<Object> ObjectManager::getObjects() const
{
  vector<Object> objects;
  map<int, Object>::const_iterator it;
  for(it = objects_.begin(); it != objects_.end(); ++it)
  {
    objects.push_back(it->second);
  }

  return objects;
}

bool ObjectManager::addObjectsFromOccupiedVoxelsFile(std::string filename)
{
  if(collision_checker_ == NULL || disable_collision_checking_)
    return false;

  if(!collision_checker_->getObjectVoxelsFromFile(filename))
    return false;

  load_objects_from_voxels_file_ = true;
  return true;
}

bool ObjectManager::writeObjectsToOccupiedVoxelsFile(std::string filename)
{
  if(collision_checker_ == NULL || disable_collision_checking_)
    return false;

  return collision_checker_->writeObjectVoxelsToFile(filename);
}

