#ifndef DEMONSTRATION_VISUALIZER
#define DEMONSTRATION_VISUALIZER

#include <QWidget>
#include <QComboBox>

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/tool_manager.h"
#include "rviz/tool.h"

#include "demonstration_visualizer_node.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "pr2_motion_recorder/FilePath.h"
#include <visualization_msgs/Marker.h>

#include <map>

class DemonstrationVisualizer : public QWidget
{
Q_OBJECT
public:
  DemonstrationVisualizer(int argc, char **argv, QWidget *parent = 0);

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

  // Load/delete a mesh file.
  void loadMesh();
  void deleteMesh();

  void selectMesh(int mesh_index);

  void interactiveMarkerMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

private:
  DemonstrationVisualizerNode node_;

  rviz::RenderPanel          *render_panel_;
  rviz::VisualizationManager *visualization_manager_;
  rviz::Display              *grid_;
  rviz::Display              *robot_model_;
  rviz::Display              *interactive_markers_;
  rviz::Display              *visualization_marker_;
  rviz::Display              *mesh_interactive_markers_;

  QComboBox                  *select_mesh_;
  std::map<int, std::string> mesh_names_;
  int                        next_mesh_id_;
  int                        selected_mesh_;

};

#endif // DEMONSTRATION_VISUALIZER_H
