#ifndef OBJECT_H
#define OBJECT_H

#include<visualization_msgs/Marker.h>
#include<geometry_msgs/Pose.h>
#include <tinyxml.h>
#include<ros/ros.h>
//#include <kdl/kdl.h>
//#include <collision_checker.h>

class Object{
  public:
    Object(visualization_msgs::Marker mesh_marker);
    Object(visualization_msgs::Marker mesh_marker, std::string sphere_list_path);
    void setPose(geometry_msgs::Pose p);
    bool checkConstraints(geometry_msgs::Pose p);
    
    visualization_msgs::Marker mesh_marker_;
    bool movable;
};

#endif
