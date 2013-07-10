#include "demonstration_visualizer/demonstration_scene_manager.h"

namespace demonstration_visualizer {

DemonstrationSceneManager::DemonstrationSceneManager()
  : goals_changed_(false), meshes_changed_(false), edit_goals_mode_(true),
    edit_meshes_mode_(true), int_marker_server_("scene_marker"), current_goal_(-1)
{
  goal_feedback_ = boost::bind(&DemonstrationSceneManager::processGoalFeedback,
			       this,
			       _1);

  mesh_feedback_ = boost::bind(&DemonstrationSceneManager::processMeshFeedback,
			       this,
			       _1);

  ros::NodeHandle nh;

  marker_pub_ = nh.advertise<visualization_msgs::Marker>("/visualization_marker", 0);
}

DemonstrationSceneManager::~DemonstrationSceneManager()
{

}

void DemonstrationSceneManager::updateScene()
{
  // First, update the goals.
  if(goals_.size() > 0 && !taskDone() && goalsChanged())
  {
    // Draw each of the goals in the current task. 
    if(editGoalsMode())
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      for(it = goals_.begin(); it != goals_.end(); ++it)
      {
	// Attach an interactive marker to control this marker.
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.header.frame_id = "/map";
	int_marker.pose = it->pose;

	// Give each interactive marker a unique name according to each goal's unique id.
	std::stringstream marker_name;
	marker_name << "goal_marker_" << it->id;

	int_marker.name = marker_name.str();

	std::stringstream marker_desc;
	marker_desc << "Goal " << it->id;
	int_marker.description = marker_desc.str();

	// Add a non-interactive control for the mesh.
	visualization_msgs::InteractiveMarkerControl control;
	control.always_visible = true;
	control.markers.push_back(*it);

	int_marker.controls.push_back(control);

	// Attach a 6-DOF control for moving the mesh around.
	attach6DOFControl(int_marker);

	int_marker_server_.insert(int_marker,
				  goal_feedback_);
	int_marker_server_.applyChanges();
      }
    }
    else // Otherwise, just draw the current goal.
    {
      // Clear the existing goal markers and interactive markers, and just draw the 
      // current goal.
      std::vector<visualization_msgs::Marker>::iterator it;
      for(it = goals_.begin(); it != goals_.end(); ++it)
      {
	// Get the name of this marker according to its id.
	std::stringstream marker_name;
	marker_name << "goal_marker_" << it->id;

	int_marker_server_.erase(marker_name.str());

	if(it->id != current_goal_)
	  it->action = visualization_msgs::Marker::DELETE;

	marker_pub_.publish(*it);
      }
      int_marker_server_.applyChanges();
    }
    
    setGoalsChanged(false);    
  }

  // Second, update the meshes. 
  if(meshes_.size() > 0 && meshesChanged())
  {
    if(editMeshesMode())
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      for(it = meshes_.begin(); it != meshes_.end(); ++it)
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
	visualizeMesh(it->id, true);
      }
    }
    else
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      // For each mesh, first remove all the interactive markers from the meshes.
      // Then, re-visualize each marker without an attached interactive marker.
      for(it = meshes_.begin(); it != meshes_.end(); ++it)
      {
	std::stringstream int_marker_name;
	int_marker_name << "mesh_marker_" << it->id;

	if(!int_marker_server_.erase(int_marker_name.str()))
	{
	  ROS_ERROR("[SceneManager] Failed to remove interactive marker on mesh %d!", it->id);
	}
	int_marker_server_.applyChanges();

	it->header.frame_id = "/map";
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::ADD;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->color.r = it->color.g = it->color.b = it->color.a = 0;
	it->mesh_use_embedded_materials = true;

	marker_pub_.publish(*it);
      }	
    }

    setMeshesChanged(false);
  }
}

int DemonstrationSceneManager::loadScene(const std::string &filename)
{
  // Clear existing meshes.
  std::vector<visualization_msgs::Marker>::iterator it;
  for(it = meshes_.begin(); it != meshes_.end(); ++it)
  {
    std::stringstream int_marker_name;
    int_marker_name << "mesh_marker_" << it->id;

    if(!int_marker_server_.erase(int_marker_name.str()))
    {
      ROS_ERROR("[SceneManager] Failed to remove interactive marker on mesh %d!", it->id);
    }
    int_marker_server_.applyChanges();

    it->header.frame_id = "/map";
    it->header.stamp = ros::Time();
    it->action = visualization_msgs::Marker::DELETE;
    it->type = visualization_msgs::Marker::MESH_RESOURCE;
    it->color.r = it->color.g = it->color.b = it->color.a = 0;
    it->mesh_use_embedded_materials = true;

    marker_pub_.publish(*it);
  }
  meshes_.clear();

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

  // Read each mesh.
  element = root_handle.FirstChild().Element();
  visualization_msgs::Marker mesh_marker;
  for(element; element; element = element->NextSiblingElement())
  {
    element->QueryIntAttribute("id", &mesh_marker.id);
    mesh_marker.ns = std::string(element->Attribute("ns"));
    mesh_marker.mesh_resource = std::string(element->Attribute("mesh_resource"));
    element->QueryDoubleAttribute("position_x", &mesh_marker.pose.position.x);
    element->QueryDoubleAttribute("position_y", &mesh_marker.pose.position.y);
    element->QueryDoubleAttribute("position_z", &mesh_marker.pose.position.z);
    element->QueryDoubleAttribute("orientation_x", &mesh_marker.pose.orientation.x);
    element->QueryDoubleAttribute("orientation_y", &mesh_marker.pose.orientation.y);
    element->QueryDoubleAttribute("orientation_z", &mesh_marker.pose.orientation.z);
    element->QueryDoubleAttribute("orientation_w", &mesh_marker.pose.orientation.w);
    element->QueryDoubleAttribute("scale_x", &mesh_marker.scale.x);
    element->QueryDoubleAttribute("scale_y", &mesh_marker.scale.y);
    element->QueryDoubleAttribute("scale_z", &mesh_marker.scale.z);

    mesh_marker.header.frame_id = "/map";
    mesh_marker.header.stamp = ros::Time();
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.color.r = mesh_marker.color.g = mesh_marker.color.b = mesh_marker.color.a = 0;
    mesh_marker.mesh_use_embedded_materials = true;

    int movable;
    element->QueryIntAttribute("movable", &movable);
    if(movable){
      std::string sphere_list_path = std::string(element->Attribute("mesh_resource"));
      Object o = Object(mesh_marker, sphere_list_path);
      object_manager.addObject(o);
    }
    else{
      Object o = Object(mesh_marker);
      object_manager.addObject(o);
    }

    //meshes_.push_back(mesh_marker);

    if(mesh_marker.id > max_mesh_id)
      max_mesh_id = mesh_marker.id;
  }

  ROS_INFO("Read %d meshes.", (int)meshes_.size());

  return max_mesh_id;
}

void DemonstrationSceneManager::saveScene(const std::string &filename)
{
  TiXmlDocument doc;
  TiXmlElement *root = new TiXmlElement("scene");
  doc.LinkEndChild(root);

  // Add each mesh to the demonstration scene description.
  std::vector<visualization_msgs::Marker>::iterator it;
  for(it = meshes_.begin(); it != meshes_.end(); ++it)
  {
    TiXmlElement *mesh = new TiXmlElement("mesh");

    mesh->SetAttribute("id", it->id);
    mesh->SetAttribute("ns", it->ns);
    mesh->SetAttribute("mesh_resource", it->mesh_resource);
    mesh->SetDoubleAttribute("position_x", it->pose.position.x);
    mesh->SetDoubleAttribute("position_y", it->pose.position.y);
    mesh->SetDoubleAttribute("position_z", it->pose.position.z);
    mesh->SetDoubleAttribute("orientation_x", it->pose.orientation.x);
    mesh->SetDoubleAttribute("orientation_y", it->pose.orientation.y);
    mesh->SetDoubleAttribute("orientation_z", it->pose.orientation.z);
    mesh->SetDoubleAttribute("orientation_w", it->pose.orientation.w);
    mesh->SetDoubleAttribute("scale_x", it->scale.x);
    mesh->SetDoubleAttribute("scale_y", it->scale.y);
    mesh->SetDoubleAttribute("scale_z", it->scale.z);
  
    root->LinkEndChild(mesh);
  }

  doc.SaveFile(filename.c_str());
}

void DemonstrationSceneManager::loadTask(const std::string &filename)
{
  // Clear existing meshes.
  goals_.clear();
  setGoalsChanged(true);

  // Load the demonstration scene from the specified file.
  TiXmlDocument doc(filename.c_str());
  if(!doc.LoadFile())
  {
    ROS_ERROR("Demonstration scene manager failed to load file %s!", filename.c_str());
    return;
  }

  TiXmlHandle doc_handle(&doc);
  TiXmlElement *element;
  TiXmlHandle root_handle(0);

  element = doc_handle.FirstChildElement().Element();
  if(!element)
  {
    // @todo error
    return;
  }

  root_handle = TiXmlHandle(element);

  ROS_INFO("Reading %s...", element->Value());

  // Read each goal.
  element = root_handle.FirstChild().Element();
  visualization_msgs::Marker goal_marker;
  goal_marker.header.frame_id = "/map";

  for(element; element; element = element->NextSiblingElement())
  {
    element->QueryIntAttribute("number", &goal_marker.id);
    goal_marker.ns = std::string(element->Attribute("ns"));
    std::string goal_desc = std::string(element->Attribute("desc"));
    element->QueryDoubleAttribute("position_x", &goal_marker.pose.position.x);
    element->QueryDoubleAttribute("position_y", &goal_marker.pose.position.y);
    element->QueryDoubleAttribute("position_z", &goal_marker.pose.position.z);
    element->QueryDoubleAttribute("orientation_x", &goal_marker.pose.orientation.x);
    element->QueryDoubleAttribute("orientation_y", &goal_marker.pose.orientation.y);
    element->QueryDoubleAttribute("orientation_z", &goal_marker.pose.orientation.z);
    element->QueryDoubleAttribute("orientation_w", &goal_marker.pose.orientation.w);

    //goal_marker.type = visualization_msgs::Marker::CUBE;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;

    // Sphere with 8cm radius.
    goal_marker.scale.x = 0.16;
    goal_marker.scale.y = 0.16;
    goal_marker.scale.z = 0.16;

    goal_marker.color.r = 0.5;
    goal_marker.color.g = 0;
    goal_marker.color.b = 0.5;
    goal_marker.color.a = 0.4;

    goals_.push_back(goal_marker);
    goal_descriptions_.push_back(goal_desc);
  }

  ROS_INFO("Read %d goals.", (int)goals_.size());

  if(getNumGoals() > 0)
    setCurrentGoal(0);
}

void DemonstrationSceneManager::saveTask(const std::string &filename)
{
  TiXmlDocument doc;
  TiXmlElement *root = new TiXmlElement("task");
  doc.LinkEndChild(root);

  // Add each goal to the task description.
  std::vector<visualization_msgs::Marker>::iterator it;
  for(it = goals_.begin(); it != goals_.end(); ++it)
  {
    TiXmlElement *goal = new TiXmlElement("goal");

    goal->SetAttribute("number", it->id);
    goal->SetAttribute("ns", it->ns);
    goal->SetAttribute("desc", goal_descriptions_[it->id]);
    goal->SetDoubleAttribute("position_x", it->pose.position.x);
    goal->SetDoubleAttribute("position_y", it->pose.position.y);
    goal->SetDoubleAttribute("position_z", it->pose.position.z);
    goal->SetDoubleAttribute("orientation_x", it->pose.orientation.x);
    goal->SetDoubleAttribute("orientation_y", it->pose.orientation.y);
    goal->SetDoubleAttribute("orientation_z", it->pose.orientation.z);
    goal->SetDoubleAttribute("orientation_w", it->pose.orientation.w);
  
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
					bool attach_interactive_marker)
{
  meshes_.push_back(marker);

  visualizeMesh(marker.id, attach_interactive_marker);
}

bool DemonstrationSceneManager::visualizeMesh(int mesh_id, bool attach_interactive_marker)
{
  std::vector<visualization_msgs::Marker>::iterator it = findMarker(meshes_, mesh_id);

  if(it == meshes_.end())
  {
    ROS_ERROR("[SceneManager] Failed to find mesh with id %d!", mesh_id);
    return false;
  }

  visualization_msgs::Marker marker = *it;

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

    int_marker_server_.insert(int_marker,
			      mesh_feedback_
			      );
    int_marker_server_.applyChanges();
  }
  else
  {
    marker_pub_.publish(marker);
  }

  return true;
}

visualization_msgs::Marker DemonstrationSceneManager::getMesh(int mesh_id)
{
  // Find the mesh marker by id.
  std::vector<visualization_msgs::Marker>::iterator it = findMarker(meshes_, mesh_id);
  
  if(it == meshes_.end())
  {
    ROS_ERROR("[SceneManager] Mesh with id %d not found!", mesh_id);
    return visualization_msgs::Marker();
  }

  return *it;
}

bool DemonstrationSceneManager::updateMeshPose(int mesh_id, const geometry_msgs::Pose &pose)
{
  // Find the marker by id.
  std::vector<visualization_msgs::Marker>::iterator it = findMarker(meshes_, mesh_id);

  if(it == meshes_.end())
    return false;

  it->pose = pose;

  return true;
}

bool DemonstrationSceneManager::updateMeshScale(int mesh_id, double x, double y, double z)
{
  // Find the mesh marker by id.
  std::vector<visualization_msgs::Marker>::iterator it = findMarker(meshes_, mesh_id);

  if(it == meshes_.end())
    return false;

  it->scale.x = x;
  it->scale.y = y;
  it->scale.z = z;

  visualizeMesh(it->id, editMeshesMode());

  return true;
}

bool DemonstrationSceneManager::removeMesh(int mesh_id)
{
  // Find the marker to delete, by id.
  std::vector<visualization_msgs::Marker>::iterator it = findMarker(meshes_, mesh_id);

  if(it == meshes_.end())
    return false;

  if(editMeshesMode())
  {
    std::stringstream marker_name;
    marker_name << "mesh_marker_" << mesh_id;

    if(!int_marker_server_.erase(marker_name.str()))
    {
      ROS_ERROR("[SceneManager] Failed to remove interactive marker %s!", marker_name.str().c_str());
    }
    int_marker_server_.applyChanges();
  }
  else
  {
    it->action = visualization_msgs::Marker::DELETE;
    visualizeMesh(it->id, false);
  }

  meshes_.erase(it);
  return true;
}

void DemonstrationSceneManager::addGoal(const geometry_msgs::Pose &pose, 
					const std::string &desc,
					const std::string &frame)
{
  setGoalsChanged(true);

  ROS_INFO("Adding goal with id %d", (int)goals_.size());

  visualization_msgs::Marker goal;
  goal.header.frame_id = frame;
  goal.header.stamp = ros::Time();
  goal.ns = "demonstration_visualizer_goal";
  goal.id = goals_.size();
  //goal.type = visualization_msgs::Marker::CUBE;
  goal.type = visualization_msgs::Marker::SPHERE;
  goal.action = visualization_msgs::Marker::ADD;

  goal.pose = pose;

  // Sphere with 8cm radius.
  goal.scale.x = 0.16;
  goal.scale.y = 0.16;
  goal.scale.z = 0.16;

  goal.color.r = 0.5;
  goal.color.g = 0;
  goal.color.b = 0.5;
  goal.color.a = 0.4;

  goals_.push_back(goal);
  goal_descriptions_.push_back(desc);

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

  goals_[goal_number].pose = pose;

  return true;
}

visualization_msgs::Marker DemonstrationSceneManager::getGoal(int goal_number)
{
  if(goal_number < 0 || goal_number >= goals_.size())
  {
    ROS_ERROR("[SceneManager] Invalid goal number!");
    return visualization_msgs::Marker();
  }

  return goals_[goal_number];
}

bool DemonstrationSceneManager::hasReachedGoal(int goal_number, 
					       const geometry_msgs::Pose &pose, 
					       double tolerance)
{
  if(getNumGoals() == 0)
    return false;

  geometry_msgs::Pose goal_pose = getGoal(goal_number).pose;
  //ROS_INFO("goal = (%f, %f, %f)", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
  double distance = std::sqrt(std::pow(goal_pose.position.x - pose.position.x, 2) +
			      std::pow(goal_pose.position.y - pose.position.y, 2) +
			      std::pow(goal_pose.position.z - pose.position.z, 2));

  if(distance < tolerance)
    return true;

  return false;
}

std::vector<visualization_msgs::Marker> DemonstrationSceneManager::getMeshes() const
{
  return meshes_;
}

std::vector<visualization_msgs::Marker> DemonstrationSceneManager::getGoals() const
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

  goal_descriptions_[goal_number] = desc;
}

std::string DemonstrationSceneManager::getGoalDescription(int goal_number) const
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager] Invalid goal number!");
    return "";
  }

  return goal_descriptions_[goal_number];
}

std::vector<visualization_msgs::Marker>::iterator DemonstrationSceneManager::findMarker(
    std::vector<visualization_msgs::Marker> &markers,
    int marker_id
  )
{
  std::vector<visualization_msgs::Marker>::iterator it;
  for(it = markers.begin(); it != markers.end(); ++it)
  {
    if(it->id == marker_id)
      break;
  }

  return it;
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
  edit_meshes_mode_ = edit;
}

void DemonstrationSceneManager::setMeshesChanged(bool changed)
{
  meshes_changed_ = changed;
}

bool DemonstrationSceneManager::meshesChanged() const
{
  return meshes_changed_;
}

bool DemonstrationSceneManager::taskDone() const
{
  return (current_goal_ >= (int)goals_.size());
}

} // namespace demonstration_visualizer
