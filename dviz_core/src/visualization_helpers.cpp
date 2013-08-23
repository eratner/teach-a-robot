#include <dviz_core/visualization_helpers.h>

namespace demonstration_visualizer {

visualization_msgs::Marker makeMeshMarker(std::string mesh_resource,
					  std::string ns,
					  int id,
					  const geometry_msgs::PoseStamped &pose_stamped,
					  const geometry_msgs::Vector3 &scale,
					  double alpha,
					  bool use_embedded_materials)
{
  visualization_msgs::Marker marker;
  
  std_msgs::ColorRGBA color;
  color.r = color.g = color.b = 0.0;
  color.a = alpha;

  marker = makeShapeMarker(visualization_msgs::Marker::MESH_RESOURCE,
			   ns,
			   id,
			   pose_stamped,
			   scale,
			   color);

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

void attach6DOFControl(visualization_msgs::InteractiveMarker &int_marker)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.orientation.w = 1;
  control.orientation.x = 1;
  control.orientation.y = 0;
  control.orientation.z = 0;
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 0;
  control.orientation.z = 1;
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);  

  control.orientation.w = 1;
  control.orientation.x = 0;
  control.orientation.y = 1;
  control.orientation.z = 0;
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
}

} // namespace demonstration_visualizer
