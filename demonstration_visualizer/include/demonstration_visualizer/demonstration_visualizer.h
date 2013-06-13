#ifndef DEMONSTRATION_VISUALIZER
#define DEMONSTRATION_VISUALIZER

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <pr2_motion_recorder/FilePath.h>
#include <pr2_simple_simulator/SetSpeed.h>
#include <visualization_msgs/Marker.h>

#include <map>

#include "demonstration_visualizer/demonstration_visualizer_node.h"
#include "demonstration_visualizer/demonstration_scene_manager.h"

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/tool_manager.h"
#include "rviz/tool.h"

#include <QWidget>
#include <QComboBox>

/* \brief A Qt-based application that provides a user interface for capturing 
          and replaying user demonstrations and creating demonstration scenes
	  from a collection of meshes.
*/
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

  // For replaying PR2 motion.
  void beginReplay();
  void endReplay();

  // Load/delete a mesh file.
  void loadMesh();
  void deleteMesh();

  // Load from/save to a demonstration scene file.
  void loadScene();
  void saveScene();

  void selectMesh(int mesh_index);

  // Called whenever an interactive marker attached to a mesh is moved. 
  void interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &);

  void setLinearSpeed(double linear);
  void setAngularSpeed(double angular);

private:
  DemonstrationVisualizerNode node_;
  DemonstrationSceneManager   demonstration_scene_manager_;

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

};

#endif // DEMONSTRATION_VISUALIZER_H
