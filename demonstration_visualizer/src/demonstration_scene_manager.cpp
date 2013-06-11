#include "demonstration_visualizer/demonstration_scene_manager.h"

DemonstrationSceneManager::DemonstrationSceneManager()
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

std::vector<visualization_msgs::Marker> DemonstrationSceneManager::getMeshes() const
{
  return meshes_;
}
