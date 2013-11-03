#ifndef COMMON_H
#define COMMON_H

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
} // namespace demonstration_visualizer

#endif // COMMON_H
