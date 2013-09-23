/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef DEMONSTRATION_VISUALIZER_CLIENT_H
#define DEMONSTRATION_VISUALIZER_CLIENT_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <map>

#include <dviz_core/demonstration_visualizer_user.h>
#include <dviz_local/helpers.h>

#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <rviz/view_manager.h>

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread.hpp>

#include <QWidget>
#include <QLabel>
#include <QKeyEvent>
#include <QTabWidget>
#include <QListWidget>
#include <QPushButton>

Q_DECLARE_METATYPE(geometry_msgs::Pose);

namespace demonstration_visualizer
{

/**
 *  @brief A Qt-based application that provides a user interface for capturing 
 *         and replaying user demonstrations and creating demonstration scenes
 *	   from a collection of meshes. This class primarily handles the user 
 *         interface.
 */
class DemonstrationVisualizerClient : public QWidget
{
Q_OBJECT
public:
  enum CameraMode { ORBIT = 0,
		    FPS,
		    TOP_DOWN,
		    AUTO,
		    TOP_DOWN_FPS,
		    GOAL };

  enum State { NORMAL = 0,
	       GRASP_SELECTION };

  DemonstrationVisualizerClient(int argc, char **argv, QWidget *parent = 0);

  virtual ~DemonstrationVisualizerClient();

  void changeState(State s);

  State getState() const;

protected:
  void keyPressEvent(QKeyEvent *event);

  void keyReleaseEvent(QKeyEvent *event);

  bool eventFilter(QObject *obj, QEvent *event);

private slots:
  /**
   * @brief Toggles the visibility of the grid.
   */
  void toggleGrid();

  /**
   * @brief Resets the state of the robot.
   */
  void resetRobot();

  void resetTask();

  void changeTool(int tool_index);

  // For recording PR2 motion.
  void beginRecording();
  void endRecording(); 

  // For replaying PR2 motion.
  void beginReplay();
  void endReplay();

  // Load/delete a mesh file.
  void loadMesh();
  void deleteMesh();
  
  void setEditSceneMode(int);

  // Load from/save to a demonstration scene file.
  void loadScene();
  void saveScene();

  // Load/save a demonstration task from file.
  void loadTask();
  void saveTask();

  void setEditGoalsMode(int);

  void selectMesh(int mesh_index);
  void scaleMesh(int value);

  void setLinearSpeed(double linear);
  void setAngularSpeed(double angular);

  // For editing tasks/goals.
  void addTaskGoal();
  void editGoalDescription(QListWidgetItem *goal);
  void showGoalsMenu(const QPoint &);

  void notifyGoalComplete(int);

  void tabChanged(int);

  void startBasicMode();
  void endBasicMode();

  // For controlling the camera.
  void updateCamera(const geometry_msgs::Pose &, const geometry_msgs::Pose &);

  void changeCameraMode(int mode);

  // For pausing/playing the simulation.
  void pauseSimulator(bool later = false);
  void playSimulator();

  void toggleZMode();

  void enableZMode();
  void disableZMode();

  void setFPSXOffset(int);
  void setFPSZOffset(int);

  void setGripperPosition(int);

  // Basic controls.
  void toggleStartStop();

  void togglePlayPause();

  void toggleGripperOrientationMode();

  void toggleCollisions();

  void acceptChangeGrasp();

  // For controlling the grasp selection mode.
  void beginGraspSelection();
  void endGraspSelection();

  void setGraspDistance(int);
  
private:
  DemonstrationVisualizerUser user_;
  boost::thread dviz_user_thread_;

  UserDemonstrationInfo user_demo_;

  rviz::RenderPanel          *render_panel_;
  rviz::VisualizationManager *visualization_manager_;
  rviz::Display              *grid_;
  rviz::Display              *robot_model_;
  rviz::Display              *robot_interactive_markers_;
  rviz::Display              *visualization_marker_;

  QComboBox                  *select_mesh_;
  std::map<int, std::string> mesh_names_;
  int                        next_mesh_id_;
  int                        selected_mesh_;
  QSlider                    *scale_mesh_slider_;

  QLabel                     *recording_icon_;
  QLabel                     *replaying_icon_;
  QListWidget                *goals_list_;
  // Tab widgets.
  QWidget                    *basic_;
  QWidget                    *advanced_;
  QWidget                    *controls_info_;

  // Basic controls.
  QPushButton                *start_stop_button_;
  QPushButton                *play_pause_button_;
  QPushButton                *z_mode_button_;
  QPushButton                *accept_change_grasp_button_;
  QPushButton                *gripper_orientation_button_;
  QPushButton                *toggle_collisions_button_;
  QSlider                    *grasp_distance_slider_;
  QSlider                    *gripper_position_slider_;

  std::vector<QPushButton *> camera_buttons_;

  CameraMode                 camera_mode_;
  CameraMode                 previous_camera_mode_;
  CameraMode                 camera_before_grasp_;

  std::string                auto_camera_state_;
  double                     auto_camera_cached_yaw_;
  double                     auto_camera_cached_pitch_;

  bool                       z_mode_;
  bool                       gripper_orientation_mode_;
  bool                       collisions_mode_;
  bool                       started_;
  bool                       playing_;
  bool                       accepted_grasp_;

  double                     x_fps_offset_;
  double                     z_fps_offset_;

  AddGoalDialog              add_goal_dialog_;

  State                      current_state_;

  int                        top_down_fps_camera_mode_;
  int                        last_top_down_fps_camera_mode_;

  bool                       grasp_selected_;

};

} // namespace demonstration_visualizer

#endif // DEMONSTRATION_VISUALIZER_CLIENT_H
