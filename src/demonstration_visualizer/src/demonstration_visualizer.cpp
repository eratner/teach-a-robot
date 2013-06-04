#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>

#include <iostream>

#include "demonstration_visualizer.h"

DemonstrationVisualizer::DemonstrationVisualizer(QWidget *parent)
 : QWidget(parent)
{
  // Initialize Rviz visualization manager and render panel.
  render_panel_ = new rviz::RenderPanel();
  visualization_manager_ = new rviz::VisualizationManager(render_panel_);

  render_panel_->initialize(visualization_manager_->getSceneManager(), visualization_manager_);
  visualization_manager_->initialize();
  visualization_manager_->startUpdate();

  // Initialize the window layout.
  QPushButton *toggle_grid = new QPushButton("Toggle grid", this);

  QVBoxLayout *controls_layout = new QVBoxLayout();
  controls_layout->addWidget(toggle_grid);

  QHBoxLayout *top_layout = new QHBoxLayout();
  top_layout->addLayout(controls_layout);
  top_layout->addWidget(render_panel_);

  // Create and display a grid.
  grid_ = visualization_manager_->createDisplay("rviz/Grid", "Grid", true);
  ROS_ASSERT(grid_ != NULL);

  

  // Connect signals to appropriate slots.
  connect(toggle_grid, SIGNAL(clicked()), this, SLOT(toggleGrid()));

  setLayout(top_layout);
}

DemonstrationVisualizer::~DemonstrationVisualizer()
{
  if(visualization_manager_ != NULL)
  {
    visualization_manager_->removeAllDisplays();
  }
  
  delete render_panel_;
  render_panel_ = 0;

  delete visualization_manager_;
  visualization_manager_ = 0;
}

void DemonstrationVisualizer::toggleGrid()
{
  if(grid_->isEnabled())
    grid_->setEnabled(false);
  else
    grid_->setEnabled(true);
}
