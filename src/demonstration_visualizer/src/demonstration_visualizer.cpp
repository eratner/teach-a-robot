#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>

#include "demonstration_visualizer.h"

DemonstrationVisualizer::DemonstrationVisualizer(QWidget *parent)
 : QWidget(parent)
{
  setWindowTitle("Demonstration Visualizer");

  resize(800, 600);

  // Initialize Rviz visualization manager and render panel.
  render_panel_ = new rviz::RenderPanel();
  visualization_manager_ = new rviz::VisualizationManager(render_panel_);

  render_panel_->initialize(visualization_manager_->getSceneManager(), visualization_manager_);
  visualization_manager_->initialize();
  visualization_manager_->startUpdate();

  // Initialize the window layout.
  QPushButton *toggle_grid = new QPushButton("Toggle Grid", this);

  QComboBox *select_tool = new QComboBox(this);
  select_tool->setInsertPolicy(QComboBox::InsertAtBottom);

  rviz::ToolManager *tool_manager = visualization_manager_->getToolManager();
  ROS_INFO("There are %d tools loaded:", tool_manager->numTools());
  for(int i = 0; i < tool_manager->numTools(); ++i)
  {
    ROS_INFO("%d. %s", i, tool_manager->getTool(i)->getClassId().toLocal8Bit().data());
    select_tool->addItem(tool_manager->getTool(i)->getClassId());
  }

  QVBoxLayout *controls_layout = new QVBoxLayout();
  controls_layout->addWidget(toggle_grid);
  controls_layout->addWidget(select_tool);

  QHBoxLayout *top_layout = new QHBoxLayout();
  top_layout->addLayout(controls_layout);
  top_layout->addWidget(render_panel_);

  // Create and display a grid.
  grid_ = visualization_manager_->createDisplay("rviz/Grid", "Grid", true);
  ROS_ASSERT(grid_ != NULL);

  // Create a robot model display.
  robot_model_ = visualization_manager_->createDisplay("rviz/RobotModel", "Robot Model", true);
  ROS_ASSERT(robot_model_ != NULL);
  ROS_INFO("Robot description: %s", 
	   robot_model_->subProp("Robot Description")->getValue().toString().toLocal8Bit().data());
  visualization_manager_->setFixedFrame("/odom_combined");
  ROS_INFO("Fixed frame: %s", 
	   visualization_manager_->getFixedFrame().toLocal8Bit().data());
  
  // Create an interactive markers display for controlling the robot. 
  interactive_markers_ = visualization_manager_->createDisplay("rviz/InteractiveMarkers", "PR2 Interactive Markers", true);
  ROS_ASSERT(interactive_markers_ != NULL);
  interactive_markers_->subProp("Update Topic")->setValue("/pr2_marker_control_transparent/update");

  // Connect signals to appropriate slots.
  connect(toggle_grid, SIGNAL(clicked()), this, SLOT(toggleGrid()));
  connect(select_tool, SIGNAL(currentIndexChanged(int)), this, SLOT(changeTool(int)));

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
  {
    grid_->setEnabled(false);
    ROS_INFO("Grid disabled.");
  }
  else
  {
    grid_->setEnabled(true);
    ROS_INFO("Grid enabled.");
  }
}

void DemonstrationVisualizer::changeTool(int tool_index)
{
  rviz::ToolManager *tool_manager = visualization_manager_->getToolManager();

  tool_manager->setCurrentTool(tool_manager->getTool(tool_index));

  ROS_INFO("Current tool changed to: %s",
	   visualization_manager_->getToolManager()->getCurrentTool()->getName().toLocal8Bit().data());  
}
