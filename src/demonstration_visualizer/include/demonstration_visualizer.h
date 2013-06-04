#ifndef DEMONSTRATION_VISUALIZER
#define DEMONSTRATION_VISUALIZER

#include <QWidget>

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"
#include "rviz/tool_manager.h"
#include "rviz/tool.h"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "pr2_motion_recorder/FilePath.h"

class DemonstrationVisualizer : public QWidget
{
Q_OBJECT
public:
  DemonstrationVisualizer(QWidget *parent = 0);

  virtual ~DemonstrationVisualizer();

private Q_SLOTS:
  void toggleGrid();
  void changeTool(int tool_index);

  // For recording PR2 joint states.
  void beginRecording();
  void endRecording(); 

private:
  rviz::RenderPanel          *render_panel_;
  rviz::VisualizationManager *visualization_manager_;
  rviz::Display              *grid_;
  rviz::Display              *robot_model_;
  rviz::Display              *interactive_markers_;

  ros::ServiceClient         begin_recording_client_;
  ros::ServiceClient         end_recording_client_;
  //ros::ServiceClient         replay_recording_client;

};

#endif // DEMONSTRATION_VISUALIZER_H
