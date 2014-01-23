#include <dviz_core/object_manager.h>
#include <sbpl_geometry_utils/SphereEncloser.h>

using namespace std;
using namespace visualization_msgs;
using namespace geometry_msgs;

namespace demonstration_visualizer
{

ObjectManager::ObjectManager(std::string rarm_filename, std::string larm_filename, int user_id, bool core)
  : user_id_(user_id), bounding_box_dimensions_(3, 0), bounding_box_origin_(3, 0), is_core_(core)
{
  rarm_file_ = rarm_filename;
  larm_file_ = larm_filename;
  enable_debug_visualizations_ = false;
  disable_collision_checking_ = true;
  load_objects_from_voxels_file_ = false;
  visualize_collision_models_ = false;
  collision_checker_ = NULL;

  ros::NodeHandle ph("~");
  ph.param("enable_debug_visualizations", enable_debug_visualizations_, false);
  ph.param("disable_collision_checking",  disable_collision_checking_, /*true*/false);
  ph.param("visualize_collision_models",  visualize_collision_models_, false);

  ROS_DEBUG("[ObjectManager%d] Initializing object manager.", user_id_);
  ROS_DEBUG("[ObjectManager%d] right_arm_file: %s", user_id_, rarm_file_.c_str());
  ROS_DEBUG("[ObjectManager%d]  left_arm_file: %s", user_id_, larm_file_.c_str());

  if(is_core_)
  {
    boost::interprocess::shared_memory_object::remove("SharedDistanceField");
  }
}

ObjectManager::~ObjectManager()
{
  // Clean up the collision checker.
  if(collision_checker_ != 0)
  {
    delete collision_checker_;
    collision_checker_ = 0;
  }

  if(is_core_)
  {
    // Remove distance fields from shared memory
    boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, "SharedDistanceField");
    ROS_INFO("[ObjectManager] DVizCore removing distance field data from shared memory.");
    segment.destroy<ShmemVectorVectorVector>("df");

    // @todo check if it exists before removing it (this is probably the cause of the boost::interprocess::interprocess_exception 
    // thrown when dviz_core is killed but no scene has been loaded)

    // Remove the shared memory
    boost::interprocess::shared_memory_object::remove("SharedDistanceField");
  }
}

bool ObjectManager::initializeCollisionChecker(const vector<double> &dims, 
					       const vector<double> &origin,
                                               const std::string &name)
{
  ROS_DEBUG("[ObjectManager%d] Initializing the collision checker.", user_id_);

  if(disable_collision_checking_)
  {
    ROS_INFO("[ObjectManager%d] Collision checking is disabled. Not initializing the collision checker.", user_id_);
    return true;
  }

  bounding_box_dimensions_ = dims;
  bounding_box_origin_ = origin;

  if(dims[0] == 0 || dims[1] == 0 || dims[2] == 0)
  {
    ROS_ERROR("[ObjectManager%d] Zero found in grid dimensions. {origin: %0.3f %0.3f %0.3f  dims: %0.3f %0.3f %0.3f}", user_id_, origin[0], origin[1], origin[2], dims[0], dims[1], dims[2]);
    return false;
  }
  
  shared_occupancy_grid_ = new sbpl_arm_planner::SharedOccupancyGrid(
    dims[0], dims[1], dims[2], 0.02, origin[0], origin[1], origin[2]);

  shared_occupancy_grid_->setReferenceFrame(resolveName("map", user_id_));

  if(collision_checker_ != 0)
  {
    ROS_ERROR("[ObjectManager%d] Deleting the previous collision checker. Initializing new one.", user_id_);
    delete collision_checker_;
    collision_checker_ = 0;
  }

  FILE* rarm_fp = NULL;
  FILE* larm_fp = NULL;

  ROS_INFO("[ObjectManager%d] Opening the arm config files to instantiate the arms.", user_id_); 
  fflush(stdout);
  // create the left & right arm models
  if((rarm_fp = fopen(rarm_file_.c_str(),"r")) == NULL)
    ROS_ERROR("[ObjectManager%d] Failed to open right arm description file.", user_id_);
  if((larm_fp=fopen(larm_file_.c_str(),"r")) == NULL)
    ROS_ERROR("[ObjectManager%d] Failed to open left arm description file.", user_id_);
  
  sbpl_arm_planner::SBPLArmModel *r_arm_model = new sbpl_arm_planner::SBPLArmModel(rarm_fp);
  sbpl_arm_planner::SBPLArmModel *l_arm_model = new sbpl_arm_planner::SBPLArmModel(larm_fp);
  r_arm_model->setResolution(0.02);
  r_arm_model->setDebugLogName("rarm");
  l_arm_model->setResolution(0.02);
  l_arm_model->setDebugLogName("larm");

  if(!r_arm_model->initKDLChainFromParamServer() || !l_arm_model->initKDLChainFromParamServer())
  {
    ROS_ERROR("[ObjectManager%d] Failed to initialize arm models.", user_id_);
    return false;
  }

  fclose(rarm_fp);
  fclose(larm_fp);

  if(!is_core_)
  {
    ROS_INFO("[ObjectManager%d] Initializing shared distance field as DVizUser.", user_id_);
    if(!shared_occupancy_grid_->initSharedDistanceField(is_core_, "df"))
    {
      ROS_ERROR("[ObjectManager%d] Failed to initialize shared distance field as DVizUser.", user_id_);
      return false;
    }
  }

  ROS_INFO("[ObjectManager%d] Constructing the collision checker.", user_id_);

  std::stringstream visualization_ns;
  visualization_ns << "dviz_user_" << user_id_ << "/collisions";
  collision_checker_ = new pr2_collision_checker::PR2CollisionSpace(r_arm_model,
								    l_arm_model,
								    shared_occupancy_grid_,
    visualization_ns.str());

  ROS_INFO("[ObjectManager%d] Initializing the collision checker.", user_id_);

  if(!collision_checker_->init("dviz_core_node"))
  {
    ROS_ERROR("[ObjectManager%d] Failed to initialize the collision checker.", user_id_);
    return false;
  }

  ROS_INFO("[ObjectManager%d] Initialized the collision checker.", user_id_);

  collision_checker_->visualizeResult(enable_debug_visualizations_);
  ROS_INFO("[ObjectManager%d] Done.", user_id_);

  return true;
}

void ObjectManager::addObject(Object o)
{
  objects_.insert(make_pair<int, Object>(o.mesh_marker_.id, o));

  if(disable_collision_checking_)
    return;

  if(is_core_ && !o.movable && !load_objects_from_voxels_file_)
  {
    ROS_INFO("[ObjectManager%d] DVizCore, placing static objects in distance field.", user_id_);
    if(!collision_checker_->addCollisionObjectMesh(o.mesh_marker_.mesh_resource,
						   o.mesh_marker_.scale,
						   o.mesh_marker_.pose,
						   o.label))
    {
      ROS_ERROR("[ObjectManager%d] Failed to add static object.", user_id_);
    }
  }
}

bool ObjectManager::addObjectFromFile(visualization_msgs::Marker &mesh_marker,
				      const std::string &collision_model_file,
				      bool movable)
{
  ROS_INFO("[ObjectManager%d] Adding object from file %s.", user_id_, collision_model_file.c_str());
  TiXmlDocument doc(collision_model_file.c_str());
  if(!doc.LoadFile())
  {
    ROS_ERROR("[ObjectManager%d] Failed to load file %s!", user_id_, collision_model_file.c_str());
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

  ROS_INFO("[ObjectManager%d] Reading %s...", user_id_, element->Value());

  // Read the mesh information.
  element = root_handle.FirstChild("object").Element();
  if(element == NULL)
  {
    ROS_ERROR("[ObjectManager%d] Element is null while trying to parse collision_model file.", user_id_);
    return false;
  }

  std::string label;
  if(element->QueryStringAttribute("label", &label) != TIXML_SUCCESS)
  {
    ROS_ERROR("[ObjectManager%d] label for the collision_model is missing.", user_id_);
    return false;
  }

  if(element->QueryStringAttribute("mesh_resource", &mesh_marker.mesh_resource) != TIXML_SUCCESS)
  {
    ROS_ERROR("[ObjectManager%d] mesh_resource for the collision_model is missing.", user_id_);
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
    ROS_DEBUG("[ObjectManager%d] movable! Reading the spheres...", user_id_);
    element = element->NextSiblingElement("sphere");

    if(!element)
    {
      if(!disable_collision_checking_)
      {
        // If no spheres found in file, generate them if cc is enabled. Takes too long otherwise.
        double enclosing_sphere_radius = 0.04;
        ROS_INFO("[ObjectManager%d] No spheres were found for '%s', so generating them instead with radius, %0.3fm", user_id_, label.c_str() , enclosing_sphere_radius); 
        std::vector<int> triangles;
        std::vector<geometry_msgs::Point> vertices; 
        if(!leatherman::getMeshComponentsFromResource(mesh_marker.mesh_resource, mesh_marker.scale, triangles, vertices))
        {
          ROS_ERROR("[ObjectManager%d] Failed to get triangles & indeces from the mesh resource.", user_id_);
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
        ROS_INFO("[ObjectManager%d] Generated %d spheres for object group '%s'.", user_id_, int(o.group_.spheres.size()), label.c_str());
      }
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
          ROS_ERROR("[ObjectManager] Failed to read y-value for sphere!");
          return false;
        }
        s.v.y(temp);
        if(element->QueryDoubleAttribute("z", &temp) != TIXML_SUCCESS)
        {
          ROS_ERROR("[ObjectManager] Failed to read z-value for sphere!");
          return false;
        }
        s.v.z(temp);
        if(element->QueryDoubleAttribute("radius", &s.radius) != TIXML_SUCCESS)
        {
          ROS_ERROR("[ObjectManager] Failed to read radius for sphere!");
          return false;
        }
        s.priority = 1;
        o.group_.spheres.push_back(s);
        ROS_DEBUG("[ObjectManager] [sphere] name: %s  x: %0.3f y: %0.3f z: %0.3f radius: %0.3f", s.name.c_str(), s.v.x(), s.v.y(), s.v.z(), s.radius);
      }
      ROS_INFO("[ObjectManager] label: %s   #_spheres: %d", o.group_.name.c_str(), int(o.group_.spheres.size()));
    }
    if(enable_debug_visualizations_ && !disable_collision_checking_)
      collision_checker_->visualizeGroup(o.group_, o.label+ boost::lexical_cast<string>(o.mesh_marker_.id), 0);
  }

  addObject(o);
  return true;
}

void ObjectManager::removeObject(int id)
{
  objects_.erase(id);
}

void ObjectManager::clearObjects()
{
  objects_.clear();
}

vector<Marker> ObjectManager::getMovedMarkers()
{
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

vector<Marker> ObjectManager::getMarkers()
{
  vector<Marker> markers;
  map<int, Object>::iterator it;
  for(it = objects_.begin(); it != objects_.end(); ++it)
  {
    markers.push_back((it->second).mesh_marker_);
  }

  return markers;
}

Marker ObjectManager::getMarker(int id)
{
  return objects_[id].mesh_marker_;
}

bool ObjectManager::checkRobotMove(const vector<double> &rangles, 
				   const vector<double> &langles, 
				   BodyPose &bp, int skip_id)
{
  double dist;
  if(collision_checker_ == NULL || disable_collision_checking_)
    return true;

  // if(visualize_collision_models_)
  //   collision_checker_->visualizeRobotCollisionModel(rangles, langles, bp, "robot_model", 0);

  //check robot against world
  ROS_DEBUG("[ObjectManager%d] Collision checking time! robot-world first", user_id_);
  if(!collision_checker_->checkRobotAgainstWorld(rangles, langles, bp, false, dist))
  {
    //ROS_INFO("COLLISION: robot <-> world");
    collision_checker_->visualizeCollision();
    return false;
  }
  //check robot against itself
  ROS_DEBUG("[ObjectManager%d] Collision checking time! robot-robot first", user_id_);
  if(!collision_checker_->checkRobotAgainstRobot(rangles, langles, bp, false, dist))
  {
    //ROS_INFO("COLLISION: robot <-> robot");
    collision_checker_->visualizeCollision();
    return false;
  }
  //check robot against all objects
  for(unsigned int i=0; i<objects_.size(); i++){
    if(int(i)==skip_id)
      continue;
    if(!collision_checker_->checkRobotAgainstGroup(rangles, langles, bp, &(objects_[i].group_), false/*true*/, false, dist))
    {
      //ROS_INFO("COLLISION robot <-> object %d", i);
      collision_checker_->visualizeCollision();
      return false;
    }  
  }
  collision_checker_->deleteCollisionVisualizations();
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

  if(visualize_collision_models_)
    collision_checker_->visualizeGroup(temp.group_, temp.group_.name, 0);

  //check the object against the environment
  if(!collision_checker_->checkGroupAgainstWorld(&(temp.group_), dist))
  {
    collision_checker_->visualizeCollision();
    return false;
  }
  //check object against robot
  if(!collision_checker_->checkRobotAgainstGroup(rangles, langles, bp, &(temp.group_), false, false, dist))
  {
    collision_checker_->visualizeCollision();
    return false;
  }  
  //check object against all other objects
  for(unsigned int i=0; i<objects_.size(); i++){
    if(int(i)==id)
      continue;
    if(!collision_checker_->checkGroupAgainstGroup(&(objects_[i].group_),&(temp.group_),dist))
    {
      collision_checker_->visualizeCollision();
      return false;
    }  
  }

  collision_checker_->deleteCollisionVisualizations();

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

int ObjectManager::getNumObjects() const
{
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

std::string ObjectManager::getObjectLabel(int id)
{
  return objects_[id].label;
}

bool ObjectManager::addObjectsFromOccupiedVoxelsFile(std::string filename)
{
  if(!is_core_)
  {
    ROS_INFO("[DVizUser] DVizUser cannot populate the distance field");
    return false;
  }

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

void ObjectManager::visualizeObjectCollisionModels()
{
  map<int, Object>::const_iterator it;
  for(it = objects_.begin(); it != objects_.end(); ++it)
  {
    Object temp = it->second;

    collision_checker_->visualizeGroup(temp.group_, temp.group_.name, 0);
  }
}

std::vector<double> ObjectManager::getBoundingBoxDimensions() const
{
  return bounding_box_dimensions_;
}

std::vector<double> ObjectManager::getBoundingBoxOrigin() const
{
  return bounding_box_origin_;
}

// @todo add a parameter to specify a (unique) identifier for each distance field
//       in shared memory
bool ObjectManager::initSharedDistanceField()
{
  if(shared_occupancy_grid_ == 0)
  {
    ROS_ERROR("[ObjectManager] In initializing shared distance field, shared occupancy grid null!");
    return false;
  }

  return shared_occupancy_grid_->initSharedDistanceField(is_core_, "df");
}

} // namespace demonstration_visualizer
