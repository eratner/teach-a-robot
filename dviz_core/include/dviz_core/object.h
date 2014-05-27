#ifndef OBJECT_H
#define OBJECT_H

#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tinyxml.h>
#include <ros/ros.h>
#include <pr2_collision_checker/pr2_collision_space.h>
#include <string>

namespace demonstration_visualizer
{

class Object
{
public:
    Object();
    Object(const visualization_msgs::Marker &mesh_marker);
    Object(const visualization_msgs::Marker &mesh_marker, const std::string &sphere_list_path);
    void setPose(const geometry_msgs::Pose &p);
    bool checkConstraints(const geometry_msgs::Pose &p);
    
    visualization_msgs::Marker mesh_marker_;
    bool movable;
    pr2_collision_checker::Group group_;
    bool redraw;
    std::string label;

};

} // namespace demonstration_visualizer

#endif
