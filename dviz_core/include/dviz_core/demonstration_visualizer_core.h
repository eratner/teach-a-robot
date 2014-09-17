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
#include <dviz_core/performance.h>

#include <boost/thread.hpp>
#include <cmath>
#include <fstream>

namespace demonstration_visualizer 
{

/**
 * @brief DVizCore is responsible for adding and killing DVizUsers, as well as 
 *        brokering communication between DVizClients and DVizUsers.
 */
class DemonstrationVisualizerCore
{
public:
  static const std::string STATS_DIRECTORY;

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
   * @brief Add a new DVizUser by spawning a new process running a DVizUser with unique ID
   *
   * @return ID of the newly created DVizUser
   */
  int addUser();

  /**
   * @brief The main run loop of DVizCore
   */
  void run();

  void writeStats();

private:
  /**
   * @brief A helper method that simply passes the Command service call
   *        along to the specified DVizUser
   */
  bool passCommandToUser(const std::string &command, 
			 std::string &response, 
			 int id,
                         const std::vector<std::string> &args = std::vector<std::string>());

  /**
   * @brief Same as passCommandToUser, but upon failure, will repeat the
   *        command until success at a given rate (note that a rate of 0
   *        will cause the command to be passed only once).
   */
  void passCommandToUserThreaded(const std::string &command,
				 std::string &response,
				 int id,
				 const std::vector<std::string> &args = std::vector<std::string>(),
				 float rate = 10.0f);

  int last_id_;
  int num_users_;
  std::map<int, ros::ServiceClient> user_command_services_;

  ros::ServiceServer command_service_;

  // For reading scenes from file and storing them in shared memory
  ObjectManager *object_manager_;
  DemonstrationSceneManager *demonstration_scene_manager_;

  // @todo replace this with some sort of lookup table to indicate
  //       which scenes have been loaded into shared memory
  //       e.g. a mapping from string name of scene to string
  //       identifier of shared memory location: 
  //            "kitchen" --> "df1"
  //            "warehouse" --> "df2"
  //            ...
  bool scene_loaded_;
  int stats_counter_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_VISUALIZER_CORE_H
