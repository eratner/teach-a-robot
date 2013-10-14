/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef DEMONSTRATION_VISUALIZER_CORE_H
#define DEMONSTRATION_VISUALIZER_CORE_H

#include <ros/ros.h>
#include <dviz_core/Command.h>
#include <dviz_core/common.h>
#include <dviz_core/object_manager.h>
#include <dviz_core/demonstration_scene_manager.h>

#include <cmath>

namespace demonstration_visualizer 
{

/**
 * @brief DVizCore is responsible for adding and killing DVizUsers, as well as 
 *        brokering communication between DVizClients and DVizUsers.
 */
class DemonstrationVisualizerCore
{
public:
  DemonstrationVisualizerCore(int argc, char **argv);

  virtual ~DemonstrationVisualizerCore();

  bool init(int argc, char **argv);

  /**
   * @brief The /dviz_command service callback for processing Command messages.
   *
   * @return true on successful execution of the command, false otherwise.
   */
  bool processCommand(dviz_core::Command::Request &, dviz_core::Command::Response &);

  /**
   * @brief An interface for providing commands without using a service protocol (i.e.
   *        by a direct method call).
   *
   * @return true on successful execution of the command, false otherwise.
   */
  bool processCommand(const std::string &command, const std::vector<std::string> &args);

  /**
   * @brief Add a new DVizUser by spawning a new process running a DVizUser with unique ID.
   *
   * @return ID of the newly created DVizUser.
   */
  int addUser();

  /**
   * @brief The main run loop of DVizCore.
   */
  void run();

private:
  /**
   * @brief A helper method that simply passes the Command service call
   *        along to the specified DVizUser.
   */
  bool passCommandToUser(const std::string &command, 
			 std::string &response, 
			 int id,
                         const std::vector<std::string> &args = std::vector<std::string>());

  int last_id_;
  int num_users_;
  std::map<int, ros::ServiceClient> user_command_services_;

  ros::ServiceServer command_service_;

  // For reading scenes from file and storing them in shared memory.
  ObjectManager *object_manager_;
  DemonstrationSceneManager *demonstration_scene_manager_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_VISUALIZER_CORE_H
