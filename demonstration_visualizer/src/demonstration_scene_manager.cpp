#include "demonstration_visualizer/demonstration_scene_manager.h"

namespace demonstration_visualizer {

const std::string DemonstrationSceneManager::GOAL_MARKER_NAMESPACE = "dviz_goal";

DemonstrationSceneManager::DemonstrationSceneManager(
    PViz *pviz,
    interactive_markers::InteractiveMarkerServer *int_marker_server,
    ObjectManager* object_manager
  )
  : goals_changed_(false), edit_goals_mode_(true),
    edit_meshes_mode_(true), int_marker_server_(int_marker_server), current_goal_(-1),
    object_manager_(object_manager), pviz_(pviz)
{
  goal_feedback_ = boost::bind(&DemonstrationSceneManager::processGoalFeedback,
			       this,
			       _1);

  mesh_feedback_ = boost::bind(&DemonstrationSceneManager::processMeshFeedback,
			       this,
			       _1);

  ros::NodeHandle nh;

  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
  ROS_INFO("[dsm] Constructor complete.");
}

DemonstrationSceneManager::~DemonstrationSceneManager()
{
  for(int i = 0; i < goals_.size(); ++i) 
  {
    if(goals_[i] != 0)
    {
      delete goals_[i];
      goals_[i] = 0;
    }
  }
}

void DemonstrationSceneManager::updateScene()
{
  // First, update the goals.
  if(goals_.size() > 0 && !taskDone() && goalsChanged())
  {
    // Draw each of the goals in the current task. 
    if(editGoalsMode())
    {
      std::vector<Goal *>::iterator it;
      for(it = goals_.begin(); it != goals_.end(); ++it)
      {
	drawGoal(*it, true);
      }
    }
    else // Otherwise, just draw the current goal.
    {
      // Clear the existing goal markers and interactive markers, and just draw the 
      // current goal.
      std::vector<Goal *>::iterator it;
      for(it = goals_.begin(); it != goals_.end(); ++it)
      {
	hideGoal(*it);

	if((*it)->getGoalNumber() == current_goal_)
	  drawGoal(*it, false);

      }
    }
    
    setGoalsChanged(false);    
  }

  // Second, update the positions of the objects. 
  std::vector<visualization_msgs::Marker> meshes = object_manager_->getMovedMarkers();  
  if(meshes.size() > 0)
  {
    if(editMeshesMode())
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	it->header.frame_id = "/map";
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::DELETE;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->mesh_use_embedded_materials = true;

	// First remove the old markers.
	marker_pub_.publish(*it);

	// Then add the markers again, but this time with interactive markers.
	it->action = visualization_msgs::Marker::ADD;
	visualizeMesh(*it, true);
      }
    }
    else
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      // For each mesh, first remove all the interactive markers from the meshes.
      // Then, re-visualize each marker without an attached interactive marker.
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	std::stringstream int_marker_name;
	int_marker_name << "mesh_marker_" << it->id;

	if(!int_marker_server_->erase(int_marker_name.str()))
	{
	  ROS_ERROR("[SceneManager] Failed to remove interactive marker on mesh %d!", it->id);
	}
	int_marker_server_->applyChanges();

	it->header.frame_id = "/map";
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::ADD;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->color.r = it->color.g = it->color.b = it->color.a = 0;
	it->mesh_use_embedded_materials = true;

	marker_pub_.publish(*it);
      }	
    }
  }
}

int DemonstrationSceneManager::loadScene(const std::string &filename)
{
  // Clear existing meshes.
  std::vector<visualization_msgs::Marker> meshes = object_manager_->getMarkers();
  std::vector<visualization_msgs::Marker>::iterator it;
  for(it = meshes.begin(); it != meshes.end(); ++it)
  {
    std::stringstream int_marker_name;
    int_marker_name << "mesh_marker_" << it->id;

    if(!int_marker_server_->erase(int_marker_name.str()))
    {
      ROS_ERROR("[SceneManager] Failed to remove interactive marker on mesh %d!", it->id);
    }
    int_marker_server_->applyChanges();

    it->header.frame_id = "/map";
    it->header.stamp = ros::Time();
    it->action = visualization_msgs::Marker::DELETE;
    it->type = visualization_msgs::Marker::MESH_RESOURCE;
    it->color.r = it->color.g = it->color.b = it->color.a = 0;
    it->mesh_use_embedded_materials = true;

    marker_pub_.publish(*it);
  }
  object_manager_->clearObjects();

  // Load the demonstration scene from the specified file.
  int max_mesh_id = -1;
  TiXmlDocument doc(filename.c_str());
  if(!doc.LoadFile())
  {
    ROS_ERROR("Demonstration scene manager failed to load file %s!", filename.c_str());
    return max_mesh_id;
  }

  TiXmlHandle doc_handle(&doc);
  TiXmlElement *element;
  TiXmlHandle root_handle(0);

  element = doc_handle.FirstChildElement().Element();
  if(!element)
  {
    // @todo error
    return max_mesh_id;
  }

  root_handle = TiXmlHandle(element);

  ROS_INFO("Reading %s...", element->Value());

  // Read the path to the collision model files.
  element = root_handle.FirstChild().Element();
  std::string package_path = ros::package::getPath("demonstration_visualizer");
  if(element == NULL)
  {
    ROS_ERROR("This is broken. XML parsing of the scene file went wrong. (element = NULL)");
    return -1;
  }
  std::string collision_model_path = std::string(element->Attribute("path"));
  ROS_INFO("Collision model path is %s", collision_model_path.c_str());

  // Read size of the environment
  element = element->NextSiblingElement();
  double origin_x, origin_y, origin_z, size_x, size_y, size_z;
  if(element->QueryDoubleAttribute("origin_x", &origin_x) != TIXML_SUCCESS){
    ROS_ERROR("origin_x is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("origin_y", &origin_y) != TIXML_SUCCESS){
    ROS_ERROR("origin_y is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("origin_z", &origin_z) != TIXML_SUCCESS){
    ROS_ERROR("origin_z is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("size_x", &size_x) != TIXML_SUCCESS){
    ROS_ERROR("size_x is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("size_y", &size_y) != TIXML_SUCCESS){
    ROS_ERROR("size_y is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("size_z", &size_z) != TIXML_SUCCESS){
    ROS_ERROR("size_z is missing");
    return -1;
  }
  vector<double> origin;
  origin.push_back(origin_x);
  origin.push_back(origin_y);
  origin.push_back(origin_z);
  vector<double> size;
  size.push_back(size_x);
  size.push_back(size_y);
  size.push_back(size_z);
  ROS_INFO("origin: %f %f %f  size: %f %f %f",
            origin[0],
            origin[1],
            origin[2],
            size[0],
            size[1],
            size[2]);
  object_manager_->initializeCollisionChecker(size, origin);

  // Read each mesh.
  element = element->NextSiblingElement();
  visualization_msgs::Marker mesh_marker;

  for(element; element; element = element->NextSiblingElement())
  {
    if(element->QueryIntAttribute("id", &mesh_marker.id) != TIXML_SUCCESS){
      ROS_ERROR("id is missing");
      return -1;
    }
    std::string label = std::string(element->Attribute("label"));
    if(element->QueryDoubleAttribute("position_x", &mesh_marker.pose.position.x) != TIXML_SUCCESS){
      ROS_ERROR("position_x is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("position_y", &mesh_marker.pose.position.y) != TIXML_SUCCESS){
      ROS_ERROR("position_y is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("position_z", &mesh_marker.pose.position.z) != TIXML_SUCCESS){
      ROS_ERROR("position_z is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("orientation_x", &mesh_marker.pose.orientation.x) != TIXML_SUCCESS){
      ROS_ERROR("orientation_x is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("orientation_y", &mesh_marker.pose.orientation.y) != TIXML_SUCCESS){
      ROS_ERROR("orientation_y is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("orientation_z", &mesh_marker.pose.orientation.z) != TIXML_SUCCESS){
      ROS_ERROR("orientation_z is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("orientation_w", &mesh_marker.pose.orientation.w) != TIXML_SUCCESS){
      ROS_ERROR("orientation_w is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("scale_x", &mesh_marker.scale.x) != TIXML_SUCCESS){
      ROS_ERROR("scale_x is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("scale_y", &mesh_marker.scale.y) != TIXML_SUCCESS){
      ROS_ERROR("scale_y is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("scale_z", &mesh_marker.scale.z) != TIXML_SUCCESS){
      ROS_ERROR("scale_z is missing");
      return -1;
    }

    mesh_marker.header.frame_id = "/map";
    mesh_marker.header.stamp = ros::Time();
    mesh_marker.ns = "demonstration_visualizer";
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.color.r = mesh_marker.color.g = mesh_marker.color.b = mesh_marker.color.a = 0;
    mesh_marker.mesh_use_embedded_materials = true;

    std::stringstream collision_model_file;
    collision_model_file << package_path << collision_model_path << "/" << label << ".xml";

    int movable;
    if(element->QueryIntAttribute("movable", &movable) != TIXML_SUCCESS){
      ROS_ERROR("movable is missing");
      return -1;
    }
    object_manager_->addObjectFromFile(mesh_marker,
				       collision_model_file.str(),
				       movable);

    visualizeMesh(mesh_marker, editMeshesMode());

    if(mesh_marker.id > max_mesh_id)
      max_mesh_id = mesh_marker.id;
  }

  ROS_INFO("[dsm] Finished getting all the elements...now setting the meshes.");

  ROS_INFO("Read %d meshes.", (int)object_manager_->getNumObjects());

  return max_mesh_id;
}

void DemonstrationSceneManager::saveScene(const std::string &filename)
{
  TiXmlDocument doc;
  TiXmlElement *root = new TiXmlElement("scene");
  doc.LinkEndChild(root);

  // Add each mesh to the demonstration scene description.
  // std::vector<visualization_msgs::Marker>::iterator it;
  // for(it = meshes_.begin(); it != meshes_.end(); ++it)
  // {
  //   TiXmlElement *mesh = new TiXmlElement("mesh");

  //   mesh->SetAttribute("id", it->id);
  //   mesh->SetAttribute("ns", it->ns);
  //   mesh->SetAttribute("mesh_resource", it->mesh_resource);
  //   mesh->SetDoubleAttribute("position_x", it->pose.position.x);
  //   mesh->SetDoubleAttribute("position_y", it->pose.position.y);
  //   mesh->SetDoubleAttribute("position_z", it->pose.position.z);
  //   mesh->SetDoubleAttribute("orientation_x", it->pose.orientation.x);
  //   mesh->SetDoubleAttribute("orientation_y", it->pose.orientation.y);
  //   mesh->SetDoubleAttribute("orientation_z", it->pose.orientation.z);
  //   mesh->SetDoubleAttribute("orientation_w", it->pose.orientation.w);
  //   mesh->SetDoubleAttribute("scale_x", it->scale.x);
  //   mesh->SetDoubleAttribute("scale_y", it->scale.y);
  //   mesh->SetDoubleAttribute("scale_z", it->scale.z);
  
  //   root->LinkEndChild(mesh);
  // }

  // doc.SaveFile(filename.c_str());
}

bool DemonstrationSceneManager::loadTask(const std::string &filename)
{
  // Clear existing meshes.
  goals_.clear();
  setGoalsChanged(true);

  // Load the demonstration scene from the specified file.
  TiXmlDocument doc(filename.c_str());
  if(!doc.LoadFile())
  {
    ROS_ERROR("[SceneManager] Failed to load file %s!", filename.c_str());
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

  ROS_INFO("Reading %s...", element->Value());

  // Read each goal.
  element = root_handle.FirstChild().Element();

  // @todo for now, we assume the user has loaded the appropriate scene file, but 
  // in the future there should be a tag at the top of the file:
  // <scene_file path="..." />
  // where path is relative to the package://demonstration_visualizer path.

  int goal_type = 0;
  int goal_number = 0;
  std::string goal_description = "";

  for(element; element; element = element->NextSiblingElement())
  {
    if(element->QueryIntAttribute("type", &goal_type) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager] Failed to read goal type!");
      return false;
    }
    if(element->QueryIntAttribute("number", &goal_number) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager] Failed to read goal number!");
      return false;
    }
    if(element->QueryStringAttribute("desc", &goal_description) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager] Failed to read goal description!");
      return false;
    }

    switch(goal_type)
    {
    case Goal::PICK_UP:
      {
	PickUpGoal *goal = new PickUpGoal(goal_number, goal_description);

	int object_id = 0;
	if(element->QueryIntAttribute("object_id", &object_id) != TIXML_SUCCESS)
	{
	  ROS_ERROR("[SceneManager] Failed to read object ID!");
	  delete goal;
	  return false;
	}

	goal->setObjectID(object_id);

	goals_.push_back(goal);

	break;
      }
    // case Goal::PLACE:
    //   break;
    default:
      break;
    }
  }

  ROS_INFO("[SceneManager] Read %d goals.", (int)goals_.size());

  for(int i = 0; i < goals_.size(); ++i)
  {
    ROS_INFO_STREAM(goals_[i]->toString());
  }

  if(getNumGoals() > 0)
    setCurrentGoal(0);

  return true;
}

void DemonstrationSceneManager::saveTask(const std::string &filename)
{
  TiXmlDocument doc;
  TiXmlElement *root = new TiXmlElement("task");
  doc.LinkEndChild(root);

  // Add each goal to the task description.
  std::vector<Goal *>::iterator it;
  for(it = goals_.begin(); it != goals_.end(); ++it)
  {
    TiXmlElement *goal = new TiXmlElement("goal");

    Goal::GoalType goal_type = (*it)->getType();
    switch(goal_type)
    {
    case Goal::PICK_UP:
      {
	PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(*it);

	goal->SetAttribute("number", pick_up_goal->getGoalNumber());
	goal->SetAttribute("type", (int)Goal::PICK_UP);
	goal->SetAttribute("desc", pick_up_goal->getDescription());
	goal->SetAttribute("object_id", pick_up_goal->getObjectID());
  
	break;
      }
    // case Goal::PLACE:
    //   break;
    default:
      break;
    }

    root->LinkEndChild(goal);
  }

  doc.SaveFile(filename.c_str());
}

void DemonstrationSceneManager::addMeshFromFile(const std::string &filename, int mesh_id)
{
  // Spawn the mesh at the origin.
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = "/map";
  pose_stamped.header.stamp = ros::Time();
  pose_stamped.pose.position.x = 0;
  pose_stamped.pose.position.y = 0;
  pose_stamped.pose.position.z = 0;
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.0;
  pose_stamped.pose.orientation.w = 1.0;  
  geometry_msgs::Vector3 scale;
  scale.x = scale.y = scale.z = 1;

  visualization_msgs::Marker marker = makeMeshMarker(filename,
						     "demonstration_visualizer",
						     mesh_id,
						     pose_stamped,
						     scale,
						     0.0,
						     true);

  addMesh(marker, true);
}

void DemonstrationSceneManager::addMesh(const visualization_msgs::Marker &marker, 
					bool attach_interactive_marker,
					const std::string &sphere_list_path)
{
  if(!sphere_list_path.empty())
  {
    // Add a movable object.
    Object o = Object(marker, sphere_list_path);
    object_manager_->addObject(o);
  }
  else
  {
    // Add a nonmovable object.
    Object o = Object(marker);
    object_manager_->addObject(o);
  }

  visualizeMesh(marker, attach_interactive_marker);
}

void DemonstrationSceneManager::visualizeMesh(const visualization_msgs::Marker &marker, 
					      bool attach_interactive_marker)
{
  if(attach_interactive_marker)
  {
    // Attach an interactive marker to control this marker.
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = "/map";
    int_marker.pose = marker.pose;

    // Give each interactive marker a unique name according to each mesh's unique id.
    std::stringstream marker_name;
    marker_name << "mesh_marker_" << marker.id;

    int_marker.name = marker_name.str();

    std::stringstream mesh_desc;
    mesh_desc << "Move " << marker_name.str();
    int_marker.description = mesh_desc.str();

    // Add a non-interactive control for the mesh.
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);

    int_marker.controls.push_back(control);

    // Attach a 6-DOF control for moving the mesh around.
    attach6DOFControl(int_marker);

    int_marker_server_->insert(int_marker,
			       mesh_feedback_
			       );
    int_marker_server_->applyChanges();
  }
  else
  {
    marker_pub_.publish(marker);
  }
}

bool DemonstrationSceneManager::updateMeshPose(int mesh_id, const geometry_msgs::Pose &pose)
{
  object_manager_->moveObject(mesh_id, pose);

  return true;
}

void DemonstrationSceneManager::updateMeshScale(int mesh_id, double x, double y, double z)
{
  object_manager_->scaleObject(mesh_id, x, y, z);

  visualizeMesh(object_manager_->getMarker(mesh_id), editMeshesMode());
}

void DemonstrationSceneManager::removeMesh(int mesh_id)
{
  visualization_msgs::Marker mesh = object_manager_->getMarker(mesh_id);

  if(editMeshesMode())
  {
    std::stringstream marker_name;
    marker_name << "mesh_marker_" << mesh_id;

    if(!int_marker_server_->erase(marker_name.str()))
    {
      ROS_ERROR("[SceneManager] Failed to remove interactive marker %s!", marker_name.str().c_str());
    }
    int_marker_server_->applyChanges();
  }
  else
  {
    mesh.action = visualization_msgs::Marker::DELETE;
    visualizeMesh(mesh, false);
  }

  object_manager_->removeObject(mesh_id);
}

void DemonstrationSceneManager::addGoal(const std::string &desc,
					Goal::GoalType type,
					int object_id)
{
  setGoalsChanged(true);

  ROS_INFO("[SceneManager] Adding goal type %s with id %d.", Goal::GoalTypeNames[type], (int)goals_.size());

  switch(type)
  {
  case Goal::PICK_UP:
    {
      PickUpGoal *goal = new PickUpGoal(goals_.size(), desc);

      geometry_msgs::Pose object_pose = object_manager_->getMarker(object_id).pose;
      goal->setGraspPose(object_pose);
      goal->setObjectID(object_id);

      goals_.push_back(goal);

      break;
    }
  case Goal::PLACE:
    {
      PlaceGoal *goal = new PlaceGoal(goals_.size(), desc, object_id);

      goals_.push_back(goal);

      break;
    }
  default:
    break;
  }

  if(current_goal_ < 0)
    current_goal_ = 0;
}

bool DemonstrationSceneManager::moveGoal(int goal_number, const geometry_msgs::Pose &pose)
{
  if(goal_number < 0 || goal_number >= goals_.size())
  {
    ROS_ERROR("Invalid goal number!");
    return false;
  }

  setGoalsChanged(true);

  switch(goals_.at(goal_number)->getType())
  {
  case Goal::PICK_UP:
    { 
      break;
    }
  case Goal::PLACE:
    {
      PlaceGoal *place_goal = static_cast<PlaceGoal *>(goals_.at(goal_number));

      place_goal->setPlacePose(pose);

      drawGoal(place_goal, editGoalsMode());

      break;
    }
  default:
    break;
  }

  return true;
}

Goal *DemonstrationSceneManager::getGoal(int goal_number)
{
  if(goal_number < 0 || goal_number >= goals_.size())
  {
    ROS_ERROR("[SceneManager] Invalid goal number!");
    return 0;
  }

  return goals_.at(goal_number);
}

bool DemonstrationSceneManager::hasReachedGoal(int goal_number, 
					       const geometry_msgs::Pose &pose, 
					       double tolerance)
{
  if(getNumGoals() == 0 || getGoal(goal_number) == 0)
    return false;

  bool reached = false;
  switch(getGoal(goal_number)->getType())
  {
  case Goal::PICK_UP:
    {
      PickUpGoal *goal = static_cast<PickUpGoal *>(getGoal(goal_number));

      geometry_msgs::Pose grasp_pose = goal->getGraspPose();

      // Combine the grasp marker pose in the map frame with the 
      // grasp distance to get the actual grasp pose in the map
      // frame.
      tf::Transform marker_in_map(tf::Quaternion(grasp_pose.orientation.x,
						 grasp_pose.orientation.y,
						 grasp_pose.orientation.z,
						 grasp_pose.orientation.w),
				  tf::Vector3(grasp_pose.position.x,
					      grasp_pose.position.y,
					      grasp_pose.position.z)
				  );

      tf::Transform gripper_in_marker(tf::Quaternion::getIdentity(),
				      tf::Vector3(-1.0*goal->getGraspDistance(),
						  0.0,
						  0.0)
				      );

      tf::Transform gripper_in_map = marker_in_map * gripper_in_marker;

      geometry_msgs::Pose grasp_pose_fixed;
      tf::quaternionTFToMsg(gripper_in_map.getRotation(), grasp_pose_fixed.orientation);
      geometry_msgs::Vector3 position;
      tf::vector3TFToMsg(gripper_in_map.getOrigin(), position);
      grasp_pose_fixed.position.x = position.x;
      grasp_pose_fixed.position.y = position.y;
      grasp_pose_fixed.position.z = position.z;

      //ROS_INFO("goal = (%f, %f, %f)", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
      double distance = std::sqrt(std::pow(grasp_pose_fixed.position.x - pose.position.x, 2) +
				  std::pow(grasp_pose_fixed.position.y - pose.position.y, 2) +
				  std::pow(grasp_pose_fixed.position.z - pose.position.z, 2));
      if(distance < 0.15)
      {
	ROS_INFO("[SceneManager] Approaching the pick up goal! (Distance = %f).", distance);
      }

      if(distance < tolerance)
	reached = true;

      break;
    }
  case Goal::PLACE:
    {
      PlaceGoal *place_goal = static_cast<PlaceGoal *>(getGoal(goal_number));

      geometry_msgs::Pose place_pose = place_goal->getPlacePose();

      double distance = std::sqrt(std::pow(place_pose.position.x - pose.position.x, 2) + 
				  std::pow(place_pose.position.y - pose.position.y, 2) +
				  std::pow(place_pose.position.z - pose.position.z, 2));

      if(distance < 0.15)
      {
	ROS_INFO("[SceneManager] Approaching the place goal! (Distance = %f).", distance);
      }

      if(distance < tolerance)
	reached = true;

      break;
    }
  default:
    break;
  }

  return reached;
}

std::vector<visualization_msgs::Marker> DemonstrationSceneManager::getMeshes() const
{
  return object_manager_->getMarkers();
}

std::vector<Object> DemonstrationSceneManager::getObjects() const
{
  return object_manager_->getObjects();
}

std::vector<Goal *> DemonstrationSceneManager::getGoals() const
{
  return goals_;
}

int DemonstrationSceneManager::getNumGoals() const
{
  return goals_.size();
}

bool DemonstrationSceneManager::goalsChanged() const
{
  return goals_changed_;
}

void DemonstrationSceneManager::setGoalsChanged(bool changed)
{
  goals_changed_ = changed;
}

void DemonstrationSceneManager::setGoalDescription(int goal_number, const std::string &desc)
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("Invalid goal number!");
    return;
  }

  goals_.at(goal_number)->setDescription(desc);
}

std::string DemonstrationSceneManager::getGoalDescription(int goal_number) const
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager] Invalid goal number!");
    return "";
  }

  return goals_.at(goal_number)->getDescription();
}

bool DemonstrationSceneManager::setGraspPose(int goal_number, const geometry_msgs::Pose &grasp)
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager] Invalid goal number!");
    return false;
  }

  if(!goals_.at(goal_number)->getType() == Goal::PICK_UP)
  {
    ROS_ERROR("[SceneManager] Can only set the grasp pose for a pick up goal!");
    return false;
  }

  PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(getGoal(goal_number));
  pick_up_goal->setGraspPose(grasp);

  return true;
}

geometry_msgs::Pose DemonstrationSceneManager::getGraspPose(int goal_number)
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager] Invalid goal number!");
    return geometry_msgs::Pose();
  }

  if(!goals_.at(goal_number)->getType() == Goal::PICK_UP)
  {
    ROS_ERROR("[SceneManager] Can only get the grasp pose for a pick up goal!");
    return geometry_msgs::Pose();
  }

  PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(getGoal(goal_number));
  return pick_up_goal->getGraspPose();  
}

void DemonstrationSceneManager::processGoalFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  int i;
  for(i = feedback->marker_name.size()-1; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  if(!moveGoal(atoi(feedback->marker_name.substr(i+1).c_str()),
	       feedback->pose))
  {
    ROS_ERROR("[SceneManager] Demonstration scene manager failed to update task goal pose!");
  }

  setGoalsChanged(true);  
}

void DemonstrationSceneManager::processMeshFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  int i;
  for(i = feedback->marker_name.size()-1; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  if(!updateMeshPose(atoi(feedback->marker_name.substr(i+1).c_str()),
		     feedback->pose)
     )
  {
    ROS_ERROR("[SceneManager] Demonstration scene manager failed to update pose of mesh!");
  }  
}

geometry_msgs::Pose DemonstrationSceneManager::getObjectPose(int object_id)
{
  return object_manager_->getMarker(object_id).pose;
}

geometry_msgs::Pose DemonstrationSceneManager::getCurrentGoalPose()
{
  if(getCurrentGoal() < 0 || getCurrentGoal() >= getNumGoals())
  {
    ROS_ERROR("[SceneManager] Invalid current goal number!");
    return geometry_msgs::Pose();
  }

  switch(goals_.at(getCurrentGoal())->getType())
  {
  case Goal::PICK_UP:
    {
      PickUpGoal *goal = static_cast<PickUpGoal *>(getGoal(getCurrentGoal()));

      return goal->getGraspPose();

      break;
    }
  case Goal::PLACE:
    {
      PlaceGoal *goal = static_cast<PlaceGoal *>(getGoal(getCurrentGoal()));

      return goal->getPlacePose();

      break;
    }
  default:
    {
      ROS_ERROR("[SceneManager] Unknown goal type!");
      return geometry_msgs::Pose();
    }
  }
}

int DemonstrationSceneManager::getCurrentGoal() const
{
  return current_goal_;
}

void DemonstrationSceneManager::setCurrentGoal(int goal)
{
  if(goal != current_goal_)
  {
    current_goal_ = goal;
    setGoalsChanged(true);
  }
}

bool DemonstrationSceneManager::editGoalsMode() const
{
  return edit_goals_mode_;
}

void DemonstrationSceneManager::setEditGoalsMode(bool edit)
{
  edit_goals_mode_ = edit;
}

bool DemonstrationSceneManager::editMeshesMode() const
{
  return edit_meshes_mode_;
}

void DemonstrationSceneManager::setEditMeshesMode(bool edit)
{
  if(edit != edit_meshes_mode_)
  {
    std::vector<visualization_msgs::Marker> meshes = object_manager_->getMarkers();
    if(edit)
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	it->header.frame_id = "/map";
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::DELETE;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->mesh_use_embedded_materials = true;

	// First remove the old markers.
	marker_pub_.publish(*it);

	// Then add the markers again, but this time with interactive markers.
	it->action = visualization_msgs::Marker::ADD;
	visualizeMesh(*it, true);
      }
    }
    else
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      // For each mesh, first remove all the interactive markers from the meshes.
      // Then, re-visualize each marker without an attached interactive marker.
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	std::stringstream int_marker_name;
	int_marker_name << "mesh_marker_" << it->id;

	if(!int_marker_server_->erase(int_marker_name.str()))
	{
	  ROS_ERROR("[SceneManager] Failed to remove interactive marker on mesh %d!", it->id);
	}
	int_marker_server_->applyChanges();

	it->header.frame_id = "/map";
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::ADD;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->color.r = it->color.g = it->color.b = it->color.a = 0;
	it->mesh_use_embedded_materials = true;

	marker_pub_.publish(*it);
      }
    }

    edit_meshes_mode_ = edit;
  }
}

bool DemonstrationSceneManager::taskDone() const
{
  return (current_goal_ >= (int)goals_.size());
}

void DemonstrationSceneManager::drawGoal(Goal *goal, bool attach_interactive_marker)
{
  switch(goal->getType())
  {
  case Goal::PICK_UP:
    {
      PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(goal);

      // Draw the shadow of the gripper around the object, where the user
      // has selected the grasp.
      if(pick_up_goal->isGraspDone())
      {
	geometry_msgs::Pose grasp_pose = pick_up_goal->getGraspPose();

	// Combine the grasp marker pose in the map frame with the 
	// grasp distance to get the actual grasp pose in the map
	// frame.
	tf::Transform marker_in_map(tf::Quaternion(grasp_pose.orientation.x,
						   grasp_pose.orientation.y,
						   grasp_pose.orientation.z,
						   grasp_pose.orientation.w),
				    tf::Vector3(grasp_pose.position.x,
						grasp_pose.position.y,
						grasp_pose.position.z)
				    );

	tf::Transform gripper_in_marker(tf::Quaternion::getIdentity(),
					tf::Vector3(-1.0*pick_up_goal->getGraspDistance(),
						    0.0,
						    0.0)
					);

	tf::Transform gripper_in_map = marker_in_map * gripper_in_marker;

	geometry_msgs::Pose grasp_pose_fixed;
	tf::quaternionTFToMsg(gripper_in_map.getRotation(), grasp_pose_fixed.orientation);
	geometry_msgs::Vector3 position;
	tf::vector3TFToMsg(gripper_in_map.getOrigin(), position);
	grasp_pose_fixed.position.x = position.x;
	grasp_pose_fixed.position.y = position.y;
	grasp_pose_fixed.position.z = position.z;

	std::vector<visualization_msgs::Marker> gripper_markers;
	pviz_->getGripperMeshesMarkerMsg(grasp_pose_fixed, 0.2, GOAL_MARKER_NAMESPACE, 1, true, gripper_markers);

	for(int i = 0; i < gripper_markers.size(); ++i)
      	{
	  gripper_markers[i].color.a = 0.3;
	  marker_pub_.publish(gripper_markers[i]);
	}
      }

      break;
    }
  case Goal::PLACE:
    {
      PlaceGoal *place_goal = static_cast<PlaceGoal *>(goal);

      // Draw a shadow of the object that can be moved around to indicate
      // where the user should place the object. 
      visualization_msgs::Marker object = object_manager_->getMarker(place_goal->getObjectID());
      
      object.header.frame_id = "/map";
      object.header.stamp = ros::Time();
      object.ns = "dviz_place_goal";
      object.id = place_goal->getGoalNumber();
      object.type = visualization_msgs::Marker::MESH_RESOURCE;
      object.action = visualization_msgs::Marker::ADD;
      object.pose = place_goal->getPlacePose();
      object.mesh_use_embedded_materials = false;
      object.color.r = 0;
      object.color.g = 0;
      object.color.b = 1;
      object.color.a = 0.4;

      if(attach_interactive_marker)
      {
	// Attach an interactive marker to control this marker.
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "/map";
	int_marker.pose = object.pose;

	// Give each interactive marker a unique name according to each goal's unique id.
	std::stringstream marker_name;
	marker_name << "goal_marker_" << object.id;

	int_marker.name = marker_name.str();

	std::stringstream mesh_desc;
	mesh_desc << "Move " << marker_name.str();
	int_marker.description = mesh_desc.str();

	// Add a non-interactive control for the mesh.
	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers.push_back(object);

	int_marker.controls.push_back(control);

	// Attach a 6-DOF control for moving the place goal around.
	attach6DOFControl(int_marker);

	int_marker_server_->insert(int_marker,
				   goal_feedback_
				   );
	int_marker_server_->applyChanges();
      }
      else
      {
	marker_pub_.publish(object);
      }

      break;
    }
  default:
    break;
  }
}

void DemonstrationSceneManager::hideGoal(Goal *goal)
{
  switch(goal->getType())
  {
  case Goal::PICK_UP:
    {
      PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(goal);

      std::vector<visualization_msgs::Marker> gripper_markers;
      pviz_->getGripperMeshesMarkerMsg(pick_up_goal->getGraspPose(), 0.2, 
				       GOAL_MARKER_NAMESPACE, 1, true, gripper_markers);

      for(int i = 0; i < gripper_markers.size(); ++i)
      {
	gripper_markers[i].action = visualization_msgs::Marker::DELETE;
	marker_pub_.publish(gripper_markers[i]);
      }

      break;
    }
  case Goal::PLACE:
    {
      PlaceGoal *place_goal = static_cast<PlaceGoal *>(goal);

      visualization_msgs::Marker object = object_manager_->getMarker(place_goal->getObjectID());
      int goal_number = place_goal->getGoalNumber();

      std::stringstream marker_name;
      marker_name << "goal_marker_" << goal_number;

      int_marker_server_->erase(marker_name.str());
      int_marker_server_->applyChanges();

      object.header.frame_id = "/map";
      object.header.stamp = ros::Time();
      object.ns = "dviz_place_goal";
      object.id = place_goal->getGoalNumber();
      object.action = visualization_msgs::Marker::DELETE;

      marker_pub_.publish(object);

      break;
    }
  default:
    break;
  }
}

} // namespace demonstration_visualizer
