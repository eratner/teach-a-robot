#ifndef DEMONSTRATION_VISUALIZER
#define DEMONSTRATION_VISUALIZER

#include <QWidget>
#include <QComboBox>

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/tool_manager.h"
#include "rviz/tool.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "pr2_motion_recorder/FilePath.h"
#include <visualization_msgs/Marker.h>

class DemonstrationVisualizer : public QWidget
{
Q_OBJECT
public:
  DemonstrationVisualizer(QWidget *parent = 0);

  virtual ~DemonstrationVisualizer();

private Q_SLOTS:
  void toggleGrid();
  void changeTool(int tool_index);

  // For recording PR2 motion.
  void beginRecording();
  void endRecording(); 

  // For replaying PR2 motion from a bag file.
  void beginReplay();
  void endReplay();

  // Load a mesh file.
  void loadMesh();

  void selectMesh(int mesh_index);

private:
  rviz::RenderPanel          *render_panel_;
  rviz::VisualizationManager *visualization_manager_;
  rviz::Display              *grid_;
  rviz::Display              *robot_model_;
  rviz::Display              *interactive_markers_;
  rviz::Display              *visualization_marker_;

  ros::ServiceClient         begin_recording_client_;
  ros::ServiceClient         end_recording_client_;
  ros::ServiceClient         begin_replay_client_;
  ros::ServiceClient         end_replay_client_;

  ros::Publisher             mesh_pub_;

  QComboBox                  *select_mesh_;
  std::vector<std::string>   mesh_names_;
  int                        selected_mesh_;

};

#endif // DEMONSTRATION_VISUALIZER_H
