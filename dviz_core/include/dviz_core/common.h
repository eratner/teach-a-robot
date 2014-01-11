#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/Pose.h>
#include <kdl/frames.hpp>

namespace demonstration_visualizer
{
  /**
   * @brief Resolves a relatively-specified name to the absolute name in the namespace
   *        of the DVizUser with the given id.
   */
  static std::string resolveName(const std::string &relative_name, int id)
  {
    std::stringstream ss;
    ss << "/dviz_user_" << id << (relative_name.substr(0, 1).compare("/") == 0 ? "" : "/")
       << relative_name;
    //ROS_INFO("resolveName(%s, %d): %s", relative_name.c_str(), id, ss.str().c_str());
    return ss.str();    
  }

  static geometry_msgs::Pose kdlFrameToPose(const KDL::Frame &frame)
  {
    geometry_msgs::Pose pose;
    pose.position.x = frame.p.x();
    pose.position.y = frame.p.y();
    pose.position.z = frame.p.z();
    
    double x, y, z, w;
    frame.M.GetQuaternion(x, y, z, w);
    pose.orientation.x = x;
    pose.orientation.y = y;
    pose.orientation.z = z;
    pose.orientation.w = w;

    return pose;
  }
} // namespace demonstration_visualizer

#endif // COMMON_H
