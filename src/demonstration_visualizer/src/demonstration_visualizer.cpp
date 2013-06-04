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
  QPushButton *info_button = new QPushButton("Info", this);

  QVBoxLayout *controls_layout = new QVBoxLayout();
  controls_layout->addWidget(info_button);

  QHBoxLayout *top_layout = new QHBoxLayout();
  top_layout->addLayout(controls_layout);
  top_layout->addWidget(render_panel_);

  // Connect signals to appropriate slots.
  connect(info_button, SIGNAL(clicked()), this, SLOT(provideInfo()));

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

void DemonstrationVisualizer::provideInfo() const
{
  std::cout << "Info requested." << std::endl;
}
