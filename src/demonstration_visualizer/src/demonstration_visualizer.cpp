#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QPushButton>
#include <QComboBox>
#include <QFileDialog>

#include "demonstration_visualizer.h"

DemonstrationVisualizer::DemonstrationVisualizer(QWidget *parent)
 : QWidget(parent)
{
  setWindowTitle("Demonstration Visualizer");

  resize(800, 600);

  // Initialize a client to the motion recording service provider.
  ros::NodeHandle nh;
  begin_recording_client_ = nh.serviceClient<pr2_motion_recorder::FilePath>("/motion_recorder/begin_recording");
  end_recording_client_ = nh.serviceClient<std_srvs::Empty>("/motion_recorder/end_recording");
  begin_replay_client_ = nh.serviceClient<pr2_motion_recorder::FilePath>("/motion_recorder/begin_replay");
  end_replay_client_ = nh.serviceClient<std_srvs::Empty>("/motion_recorder/end_replay");
  
  mesh_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);

  // Initialize Rviz visualization manager and render panel.
  render_panel_ = new rviz::RenderPanel();
  visualization_manager_ = new rviz::VisualizationManager(render_panel_);

  render_panel_->initialize(visualization_manager_->getSceneManager(), visualization_manager_);
  visualization_manager_->initialize();
  visualization_manager_->startUpdate();

  // Initialize the window layout.
  QPushButton *toggle_grid = new QPushButton("Toggle Grid", this);

  // Recording control panel.
  QHBoxLayout *recording_controls = new QHBoxLayout();
  QPushButton *begin_recording = new QPushButton("Begin Recording", this);
  recording_controls->addWidget(begin_recording);
  QPushButton *end_recording = new QPushButton("End Recording", this);
  recording_controls->addWidget(end_recording);

  // Replay control panel.
  QHBoxLayout *replay_controls = new QHBoxLayout();
  QPushButton *begin_replay = new QPushButton("Begin Replay", this);
  replay_controls->addWidget(begin_replay);
  QPushButton *end_replay = new QPushButton("End Replay", this);
  replay_controls->addWidget(end_replay);

  // Select tool control panel
  QComboBox *select_tool = new QComboBox(this);
  select_tool->setInsertPolicy(QComboBox::InsertAtBottom);

  rviz::ToolManager *tool_manager = visualization_manager_->getToolManager();
  ROS_INFO("There are %d tools loaded:", tool_manager->numTools());
  for(int i = 0; i < tool_manager->numTools(); ++i)
  {
    ROS_INFO("%d. %s", i, tool_manager->getTool(i)->getClassId().toLocal8Bit().data());
    select_tool->addItem(tool_manager->getTool(i)->getClassId());
  }

  // Load mesh control panel.
  QPushButton *load_mesh = new QPushButton("Load Mesh");

  QVBoxLayout *controls_layout = new QVBoxLayout();
  controls_layout->addWidget(toggle_grid);
  controls_layout->addLayout(recording_controls);
  controls_layout->addLayout(replay_controls);
  controls_layout->addWidget(load_mesh);
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

  // Create a visualization marker for loading meshes of environments.
  visualization_marker_ = visualization_manager_->createDisplay("rviz/Marker", "Mesh", true);
  ROS_ASSERT(visualization_marker_ != NULL);

  // Connect signals to appropriate slots.
  connect(toggle_grid, SIGNAL(clicked()), this, SLOT(toggleGrid()));
  connect(begin_recording, SIGNAL(clicked()), this, SLOT(beginRecording()));
  connect(end_recording, SIGNAL(clicked()), this, SLOT(endRecording()));
  connect(select_tool, SIGNAL(currentIndexChanged(int)), this, SLOT(changeTool(int)));
  //connect(begin_replay, SIGNAL(clicked()), this, SLOT(beginReplay()));
  //connect(end_replay, SIGNAL(clicked()), this, SLOT(endReplay()));
  connect(load_mesh, SIGNAL(clicked()), this, SLOT(loadMesh()));

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

void DemonstrationVisualizer::beginRecording()
{
  QString directory = QFileDialog::getExistingDirectory(this,
							tr("Open Directory"),
							"/home",
							QFileDialog::ShowDirsOnly);

  pr2_motion_recorder::FilePath srv;
  srv.request.file_path = directory.toStdString();
  if(!begin_recording_client_.call(srv))
  {
    ROS_ERROR("Failed to call service /motion_recorder/begin_recording.");
    return;
  }

  ROS_INFO("Recording to %s", directory.toLocal8Bit().data());
}

void DemonstrationVisualizer::endRecording()
{
  std_srvs::Empty srv;
  if(!end_recording_client_.call(srv))
  {
    ROS_ERROR("Failed to call service /motion_recorder/end_recording.");
    return;
  }

  ROS_INFO("Recording has ended.");
}

void DemonstrationVisualizer::loadMesh()
{
  QString filename = QFileDialog::getOpenFileName(this, 
						  tr("Open Mesh File"),
						  "/home",
						  tr("Mesh Files (*.dae)"));

  std::stringstream resource_path;
  resource_path << "file://" << filename.toStdString();
  ROS_INFO("Loading mesh from file %s.", resource_path.str().c_str());

  visualization_msgs::Marker marker;
  marker.header.frame_id = "/odom_combined";
  marker.header.stamp = ros::Time();
  marker.ns = "demo_vis";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 1;
  marker.pose.position.y = 1;
  marker.pose.position.z = 1;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 0.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.mesh_resource = resource_path.str();
  marker.mesh_use_embedded_materials = true;
  mesh_pub_.publish(marker);  
}
