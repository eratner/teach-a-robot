/**
 * @author Ellis Ratner
 * @date June 2013
 */
#ifndef DEMONSTRATION_VISUALIZER
#define DEMONSTRATION_VISUALIZER

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pr2_simple_simulator/FilePath.h>
#include <pr2_simple_simulator/SetSpeed.h>
#include <visualization_msgs/Marker.h>

#include <map>

#include "demonstration_visualizer/demonstration_visualizer_node.h"

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/tool_manager.h"
#include "rviz/tool.h"
#include "rviz/view_manager.h"

#include <QWidget>
#include <QComboBox>
#include <QLabel>
#include <QKeyEvent>
#include <QTabWidget>
#include <QListWidget>
#include <QPushButton>

struct UserDemonstrationInfo
{
  UserDemonstrationInfo()
  : start_time_(), end_time_(), started_(false), goals_completed_(0)
  {

  }

  void reset()
  {
    start_time_ = ros::Time();
    end_time_ = ros::Time();
  }

  void start()
  {
    start_time_ = ros::Time::now();
    started_ = true;
  }
  
  ros::Duration stop()
  {
    if(started_)
    {
      started_ = false;
      end_time_ = ros::Time::now();
      return (end_time_ - start_time_);
    }

    return ros::Duration(0.0);
  }

  ros::Time start_time_;
  ros::Time end_time_;
  bool started_;
  int goals_completed_;
};

/**
 *  @brief A Qt-based application that provides a user interface for capturing 
 *         and replaying user demonstrations and creating demonstration scenes
 *	   from a collection of meshes. This class primarily handles the user 
 *         interface.
 */
class DemonstrationVisualizer : public QWidget
{
Q_OBJECT
public:
  DemonstrationVisualizer(int argc, char **argv, QWidget *parent = 0);

  virtual ~DemonstrationVisualizer();

protected:
  void keyPressEvent(QKeyEvent *event);

  void keyReleaseEvent(QKeyEvent *event);

  bool eventFilter(QObject *obj, QEvent *event);

private Q_SLOTS:
  /**
   * @brief Toggles the visibility of the grid.
   */
  void toggleGrid();

  /**
   * @brief Resets the state of the robot.
   */
  void resetRobot();

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

  // Load from/save to a demonstration scene file.
  void loadScene();
  void saveScene();

  // Load/save a demonstration task from file.
  void loadTask();
  void saveTask();

  void setEditGoalsMode(int);

  void selectMesh(int mesh_index);

  void setLinearSpeed(double linear);
  void setAngularSpeed(double angular);

  // For editing tasks/goals.
  void addTaskGoal();
  void editGoalDescription(QListWidgetItem *goal);

  void notifyGoalComplete(int);

  void tabChanged(int);

  void startBasicMode();
  void endBasicMode();
  
private:
  DemonstrationVisualizerNode node_;
  UserDemonstrationInfo user_demo_;

  rviz::RenderPanel          *render_panel_;
  rviz::VisualizationManager *visualization_manager_;
  rviz::Display              *grid_;
  rviz::Display              *robot_model_;
  rviz::Display              *robot_interactive_markers_;
  rviz::Display              *visualization_marker_;
  rviz::Display              *mesh_interactive_markers_;

  QComboBox                  *select_mesh_;
  std::map<int, std::string> mesh_names_;
  int                        next_mesh_id_;
  int                        selected_mesh_;

  QLabel                     *recording_icon_;
  QLabel                     *replaying_icon_;
  QListWidget                *goals_list_;
  QWidget                    *basic_;
  QWidget                    *advanced_;

  // Basic controls.
  QPushButton                *start_button_;
  QPushButton                *end_button_;

};

#endif // DEMONSTRATION_VISUALIZER_H
