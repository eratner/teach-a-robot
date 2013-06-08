#include "visualization_helpers.h"

visualization_msgs::Marker makeMeshMarker(std::string mesh_resource,
					  std::string ns,
					  int id,
					  const geometry_msgs::PoseStamped &pose_stamped,
					  const geometry_msgs::Vector3 &scale,
					  float alpha,
					  bool use_embedded_materials)
{
  visualization_msgs::Marker marker;
  
  marker.header = pose_stamped.header;
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose_stamped.pose;
  marker.scale = scale;
  marker.color.a = alpha;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.mesh_resource = mesh_resource;
  marker.mesh_use_embedded_materials = use_embedded_materials;

  return marker;
}

visualization_msgs::Marker makeShapeMarker(int type,
					   std::string ns,
					   int id,
					   const geometry_msgs::PoseStamped &pose_stamped,
					   const geometry_msgs::Vector3 &scale,
					   const std_msgs::ColorRGBA &color)
{
  visualization_msgs::Marker marker;
  marker.header = pose_stamped.header;
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose_stamped.pose;
  marker.scale = scale;
  marker.color = color;

  return marker;
}
