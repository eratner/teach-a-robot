#include "demonstration_visualizer/demonstration_scene_manager.h"

DemonstrationSceneManager::DemonstrationSceneManager()
  : goals_changed_(false)
{

}

DemonstrationSceneManager::~DemonstrationSceneManager()
{

}

void DemonstrationSceneManager::loadScene(const std::string &filename)
{
  // Clear existing meshes.
  meshes_.clear();

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

    meshes_.push_back(mesh_marker);
  }

  ROS_INFO("Read %d meshes.", (int)meshes_.size());
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

    goal_marker.type = visualization_msgs::Marker::CUBE;
    goal_marker.action = visualization_msgs::Marker::ADD;

    // Cube with 10cm sides.
    goal_marker.scale.x = 0.1;
    goal_marker.scale.y = 0.1;
    goal_marker.scale.z = 0.1;

    goal_marker.color.r = 0;
    goal_marker.color.g = 0;
    goal_marker.color.b = 1;
    goal_marker.color.a = 0.4;

    goals_.push_back(goal_marker);
    goal_descriptions_.push_back(goal_desc);
  }

  ROS_INFO("Read %d goals.", (int)goals_.size());
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

void DemonstrationSceneManager::addMesh(const visualization_msgs::Marker &marker)
{
  meshes_.push_back(marker);
}

bool DemonstrationSceneManager::updateMeshPose(int mesh_id, const geometry_msgs::Pose &pose)
{
  // Find the marker by id.
  std::vector<visualization_msgs::Marker>::iterator it;
  for(it = meshes_.begin(); it != meshes_.end(); ++it)
  {
    if(it->id == mesh_id)
      break;
  }

  if(it == meshes_.end())
    return false;

  it->pose = pose;

  return true;
}

bool DemonstrationSceneManager::removeMesh(int mesh_id)
{
  // Find the marker to delete, by id.
  std::vector<visualization_msgs::Marker>::iterator it;
  for(it = meshes_.begin(); it != meshes_.end(); ++it)
  {
    if(it->id == mesh_id)
      break;
  }

  if(it == meshes_.end())
    return false;

  meshes_.erase(it);
  return true;
}

void DemonstrationSceneManager::addGoal(const geometry_msgs::Pose &pose, 
					const std::string &desc,
					const std::string &frame)
{
  setGoalsChanged(true);

  visualization_msgs::Marker goal;
  goal.header.frame_id = frame;
  goal.header.stamp = ros::Time();
  goal.ns = "demonstration_visualizer_goal";
  goal.id = goals_.size();
  goal.type = visualization_msgs::Marker::CUBE;
  goal.action = visualization_msgs::Marker::ADD;

  goal.pose = pose;

  // Cube with 10cm sides.
  goal.scale.x = 0.1;
  goal.scale.y = 0.1;
  goal.scale.z = 0.1;

  goal.color.r = 0;
  goal.color.g = 0;
  goal.color.b = 1;
  goal.color.a = 0.4;

  goals_.push_back(goal);
  goal_descriptions_.push_back(desc);
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
    ROS_ERROR("Invalid goal number!");
    return visualization_msgs::Marker();
  }

  return goals_[goal_number];
}

bool DemonstrationSceneManager::hasReachedGoal(int goal_number, const geometry_msgs::Pose &pose)
{
  geometry_msgs::Pose goal_pose = getGoal(goal_number).pose;
  double distance = std::sqrt(std::pow(goal_pose.position.x - pose.position.x, 2) +
			      std::pow(goal_pose.position.y - pose.position.y, 2) +
			      std::pow(goal_pose.position.z - pose.position.z, 2));
  // @todo add angle.

  // Tolerance of 5cm.
  if(distance < 0.05)
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
    ROS_ERROR("Invalid goal number!");
    return "";
  }

  return goal_descriptions_[goal_number];
}
