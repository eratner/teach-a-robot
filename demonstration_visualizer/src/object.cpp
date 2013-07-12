#include<demonstration_visualizer/object.h>
#include <boost/lexical_cast.hpp>
#include <kdl/frames.hpp>

Object::Object() {
  movable = false;
}

Object::Object(visualization_msgs::Marker mesh_marker){
  mesh_marker_ = mesh_marker;
  movable = false;
  redraw = true;
}

Object::Object(visualization_msgs::Marker mesh_marker, std::string sphere_list_path){
  mesh_marker_ = mesh_marker;
  movable = true;
  redraw = true;

  TiXmlDocument doc(sphere_list_path.c_str());
  if(!doc.LoadFile()){
    ROS_ERROR("Object failed to load file %s!", sphere_list_path.c_str());
    return;
  }

  TiXmlHandle doc_handle(&doc);
  TiXmlElement *element;
  TiXmlHandle root_handle(0);

  element = doc_handle.FirstChildElement().Element();
  if(!element){
    // @todo error
    return;
  }

  root_handle = TiXmlHandle(element);

  ROS_INFO("Reading %s...", element->Value());

  // read each 
  element = root_handle.FirstChild().Element();
  for(element; element; element = element->NextSiblingElement()){
    pr2_collision_checker::Sphere s;
    int id;
    element->QueryIntAttribute("id", &id);
    s.name = boost::lexical_cast<string>(id);
    double temp;
    element->QueryDoubleAttribute("x", &temp);
    s.v.x(temp);
    element->QueryDoubleAttribute("y", &temp);
    s.v.y(temp);
    element->QueryDoubleAttribute("z", &temp);
    s.v.z(temp);
    element->QueryDoubleAttribute("radius", &s.radius);
    s.priority = 1;
    group_.spheres.push_back(s);
  }
  group_.name = sphere_list_path;

}

void Object::setPose(geometry_msgs::Pose p){
  mesh_marker_.pose = p;

  if(movable){
    KDL::Vector v(p.position.x,p.position.y,p.position.z);
    group_.f.p = v;
    group_.f.M.Quaternion(p.orientation.x,
                          p.orientation.y,
                          p.orientation.z,
                          p.orientation.w);
  }
}

bool Object::checkConstraints(geometry_msgs::Pose p){
  return true;
}

