#ifndef VISUALIZATION_HELPERS_H
#define VISUALIZATION_HELPERS_H

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>

#include <string>

namespace demonstration_visualizer {

visualization_msgs::Marker makeMeshMarker(std::string mesh_resource,
					  std::string ns,
					  int id,
					  const geometry_msgs::PoseStamped &pose_stamped,
					  const geometry_msgs::Vector3 &scale,
					  double alpha = 1.0,
					  bool use_embedded_materials = true);

visualization_msgs::Marker makeShapeMarker(int type,
					   std::string ns,
					   int id,
					   const geometry_msgs::PoseStamped &pose_stamped,
					   const geometry_msgs::Vector3 &scale,
					   const std_msgs::ColorRGBA &color);

void attach6DOFControl(visualization_msgs::InteractiveMarker &int_marker);

} // namespace demonstration_visualizer

#endif // VISUALIZATION_HELPERS_H
