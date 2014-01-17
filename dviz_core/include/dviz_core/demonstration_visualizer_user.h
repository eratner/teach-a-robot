
#ifndef DEMONSTRATION_VISUALIZER_USER_H
#define DEMONSTRATION_VISUALIZER_USER_H

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>

#include <dviz_core/demonstration_scene_manager.h>
#include <dviz_core/pr2_simulator.h>
#include <dviz_core/object_manager.h>
#include <dviz_core/motion_recorder.h>
#include <dviz_core/common.h>
#include <dviz_core/Command.h>
#include <dviz_core/Goal.h>
#include <dviz_core/Task.h>

#include <boost/thread/mutex.hpp>

namespace demonstration_visualizer
{

/**
 * @brief DVizUser provides all the back-end functionality of performing 
 *        and capturing user demonstrations in a simulated environment.
 */
class DemonstrationVisualizerUser
{
public:
  DemonstrationVisualizerUser(int argc, char **argv, int id, bool web = true);

  ~DemonstrationVisualizerUser();

  /**
   * @brief The main run loop of DVizUser.
   */
  void run();

  /**
   * @brief The service callback for processing Command messages.
   *
   * @return true on successful execution of the command, false otherwise.
   */
  bool processCommand(dviz_core::Command::Request &, dviz_core::Command::Response &);

  /**
   * @brief
   *
   * @return true on successful execution of the command, false otherwise.
   */
  bool processCommand(const std::string &command, const std::vector<std::string> &args);

  /**
   * @brief Updates the goals and tasks, and monitors if goals have been completed 
   *        or tasks change. Also publishes the current task and task state for 
   *        DVizClients. 
   */
  void updateGoalsAndTask();

  /**
   * @brief Show the base path of a recorded motion.
   */
  void showBasePath(const std::string &filename = std::string());

  /**
   * @brief Returns a pointer to the Demonstration Scene Manager for this 
   *        DVizUser. (be careful)
   */
  DemonstrationSceneManager *getSceneManager();

  /**
   * @brief Set the frame rate at which the main loop runs.
   */
  void setFrameRate(double);

  /**
   * @brief Get the frame rate at which the main loop runs.
   */
  double getFrameRate() const;

  /**
   * @brief Get statistics on this DVizUser process, such as CPU and memory usage.
   */
  void getUserProcessInfo();

  /**
   * @brief Encapsulates a series of steps that must be taken once a goal has completed.
   */
  void goalCompleted();

  /**
   * @brief Shows a gripper marker centered at the given goal (if that goal is of type
   *        PICK_UP), which a user may interact with via interactive markers
   */
  bool showInteractiveGripper(int);

  bool hideInteractiveGripper(int);

  void gripperMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void graspArrowMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  /**
   * @brief Process key events sent from the client
   * @param key The key code associated with the event
   * @param type The type of key event (e.g. press or release)
   */
  void processKeyEvent(int key, int type);
  
  /**
   * @brief If a task has been loaded, reset the task (move the
   *        objects back to their initial locations
   */
  void resetTask();

private:
  /**
   * @brief Initializes ROS and names the DVizUser node with its
   *        unique identifier
   *
   * @return true on success; false on failure
   */
  bool init(int argc, char **argv);

  /**
   * @brief Updates the interactive gripper marker array for the 
   *        given (PICK_UP) goal
   */
  void updateGripperMarkers(int goal_number);

  int id_;
  bool ok_;
  bool web_;
  double frame_rate_;
  bool frame_rate_changed_;
  bool accepted_grasp_;
  ros::ServiceServer command_service_;

  ros::Publisher task_pub_;
  ros::Publisher marker_array_pub_;

  interactive_markers::InteractiveMarkerServer *int_marker_server_;
  PViz *pviz_;
  ObjectManager *object_manager_;
  MotionRecorder *recorder_;
  DemonstrationSceneManager *demonstration_scene_manager_;
  PR2Simulator *simulator_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_VISUALIZER_USER_H
