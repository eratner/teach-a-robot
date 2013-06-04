#ifndef DEMONSTRATION_VISUALIZER
#define DEMONSTRATION_VISUALIZER

#include <QWidget>

#include "rviz/render_panel.h"
#include "rviz/visualization_manager.h"
#include "rviz/display.h"

class DemonstrationVisualizer : public QWidget
{
Q_OBJECT
public:
  DemonstrationVisualizer(QWidget *parent = 0);

  virtual ~DemonstrationVisualizer();

private Q_SLOTS:
  void toggleGrid();

private:
  rviz::RenderPanel          *render_panel_;
  rviz::VisualizationManager *visualization_manager_;
  rviz::Display              *grid_;
  rviz::Display              *robot_model_;

};

#endif // DEMONSTRATION_VISUALIZER_H
