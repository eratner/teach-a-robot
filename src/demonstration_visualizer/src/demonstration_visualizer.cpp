#include "demonstration_visualizer.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QFileDialog>

DemonstrationVisualizer::DemonstrationVisualizer(int argc, char **argv, QWidget *parent)
 : QWidget(parent), node_(argc, argv)
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

  // Select tool control panel.
  QComboBox *select_tool = new QComboBox();
  select_tool->setInsertPolicy(QComboBox::InsertAtBottom);
  select_tool->addItem("Select Tool...");

  // Select possible meshes.
  select_mesh_ = new QComboBox();
  select_mesh_->setInsertPolicy(QComboBox::InsertAtBottom);
  select_mesh_->addItem("Select Mesh...");

  rviz::ToolManager *tool_manager = visualization_manager_->getToolManager();
  ROS_INFO("There are %d tools loaded:", tool_manager->numTools());
  for(int i = 0; i < tool_manager->numTools(); ++i)
  {
    ROS_INFO("%d. %s", i, tool_manager->getTool(i)->getClassId().toLocal8Bit().data());
    select_tool->addItem(tool_manager->getTool(i)->getClassId());
  }

  // Load mesh control panel.
  QHBoxLayout *mesh_controls = new QHBoxLayout();
  QPushButton *load_mesh = new QPushButton("Load Mesh");
  mesh_controls->addWidget(load_mesh);
  QPushButton *delete_mesh = new QPushButton("Delete Mesh");
  mesh_controls->addWidget(delete_mesh);

  QVBoxLayout *controls_layout = new QVBoxLayout();
  controls_layout->addWidget(toggle_grid);
  controls_layout->addLayout(recording_controls);
  controls_layout->addLayout(replay_controls);
  controls_layout->addLayout(mesh_controls);
  controls_layout->addWidget(select_mesh_);
  controls_layout->addWidget(select_tool);

  QGridLayout *window_layout = new QGridLayout();
  window_layout->addLayout(controls_layout, 0, 0, 1, 1);
  window_layout->addWidget(render_panel_, 0, 1, 2, 3);

  // Create and display a grid.
  grid_ = visualization_manager_->createDisplay("rviz/Grid", "Grid", true);
  ROS_ASSERT(grid_ != NULL);

  // Create a robot model display.
  robot_model_ = visualization_manager_->createDisplay("rviz/RobotModel", "Robot Model", true);
  ROS_ASSERT(robot_model_ != NULL);
  ROS_INFO("Robot description: %s", 
	   robot_model_->subProp("Robot Description")->getValue().toString().toLocal8Bit().data());
  visualization_manager_->setFixedFrame("/map");
  ROS_INFO("Fixed frame: %s", 
	   visualization_manager_->getFixedFrame().toLocal8Bit().data());
  
  // Create an interactive markers display for controlling the robot. 
  interactive_markers_ = visualization_manager_->createDisplay("rviz/InteractiveMarkers", "PR2 Interactive Markers", true);
  ROS_ASSERT(interactive_markers_ != NULL);
  interactive_markers_->subProp("Update Topic")->setValue("/pr2_marker_control_transparent/update");

  // Create a visualization marker for loading meshes of environments.
  visualization_marker_ = visualization_manager_->createDisplay("rviz/Marker", "Mesh", true);
  visualization_marker_->subProp("Marker Topic")->setValue("/demonstration_visualizer/visualization_marker");
  ROS_ASSERT(visualization_marker_ != NULL);

  // Create an interactive markers display for moving meshes around the scene.
  mesh_interactive_markers_ = visualization_manager_->createDisplay("rviz/InteractiveMarkers",
								    "Mesh Interactive Markers",
								    true);
  ROS_ASSERT(mesh_interactive_markers_ != NULL);
  mesh_interactive_markers_->subProp("Update Topic")->setValue("/mesh_marker/update");

  // Create an interactive marker to allow the user to interact with the base of the robot.
  base_movement_marker_ = visualization_manager_->createDisplay("rviz/InteractiveMarkers",
								"Move Base Marker",
								true);
  ROS_ASSERT(base_movement_marker_ != NULL);
  base_movement_marker_->subProp("Update Topic")->setValue("/base_marker/update");

  // Connect signals to appropriate slots.
  connect(toggle_grid, SIGNAL(clicked()), this, SLOT(toggleGrid()));
  connect(begin_recording, SIGNAL(clicked()), this, SLOT(beginRecording()));
  connect(end_recording, SIGNAL(clicked()), this, SLOT(endRecording()));
  connect(select_tool, SIGNAL(currentIndexChanged(int)), this, SLOT(changeTool(int)));
  connect(begin_replay, SIGNAL(clicked()), this, SLOT(beginReplay()));
  connect(end_replay, SIGNAL(clicked()), this, SLOT(endReplay()));
  connect(load_mesh, SIGNAL(clicked()), this, SLOT(loadMesh()));
  connect(select_mesh_, SIGNAL(currentIndexChanged(int)), this, SLOT(selectMesh(int)));
  connect(delete_mesh, SIGNAL(clicked()), this, SLOT(deleteMesh()));
  connect(&node_, SIGNAL(interactiveMarkerMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)),
	  this, SLOT(interactiveMarkerMoved(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)));

  next_mesh_id_ = 2;
  selected_mesh_ = -1;

  setLayout(window_layout);
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
  if(tool_index == 0)
    return;

  rviz::ToolManager *tool_manager = visualization_manager_->getToolManager();

  tool_manager->setCurrentTool(tool_manager->getTool(tool_index-1));

  ROS_INFO("Current tool changed to: %s",
	   visualization_manager_->getToolManager()->getCurrentTool()->getName().toLocal8Bit().data());  
}

void DemonstrationVisualizer::beginRecording()
{
  QString directory = QFileDialog::getExistingDirectory(this,
							tr("Open Directory"),
							"/home",
							QFileDialog::ShowDirsOnly);

  if(directory.isEmpty())
  {
    ROS_INFO("No directory selected.");
    return;
  }

  pr2_motion_recorder::FilePath srv;
  srv.request.file_path = directory.toStdString();
  if(!node_.beginRecording(srv))
  {
    ROS_ERROR("Failed to call service /motion_recorder/begin_recording.");
    return;
  }

  ROS_INFO("Recording to %s", directory.toLocal8Bit().data());
}

void DemonstrationVisualizer::endRecording()
{
  std_srvs::Empty srv;
  if(!node_.endRecording(srv))
  {
    ROS_ERROR("Failed to call service /motion_recorder/end_recording.");
    return;
  }

  ROS_INFO("Recording has ended.");
}

void DemonstrationVisualizer::beginReplay()
{
  QString filename = QFileDialog::getOpenFileName(this,
						  tr("Replay Bag File"),
						  "/home",
						  tr("Bag Files *.bag"));

  if(filename.isEmpty())
  {
    ROS_INFO("No directory selected.");
    return;
  }

  pr2_motion_recorder::FilePath srv;
  srv.request.file_path = filename.toStdString();
  if(!node_.beginReplay(srv))
  {
    ROS_ERROR("Failed to call service /motion_recorder/begin_replay.");
    return;
  }

  ROS_INFO("Replaying from file %s", filename.toLocal8Bit().data());
}

void DemonstrationVisualizer::endReplay()
{
  //@todo node_.endReplay(srv); ...
}

void DemonstrationVisualizer::loadMesh()
{
  QString filename = QFileDialog::getOpenFileName(this, 
						  tr("Open Mesh File"),
						  "/home",
						  tr("Mesh Files (*.dae *.stl *.mesh)"));

  if(filename.isEmpty())
  {
    ROS_INFO("No file selected.");
    return;
  }

  std::stringstream resource_path;
  resource_path << "file://" << filename.toStdString();
  ROS_INFO("Loading mesh from file %s.", resource_path.str().c_str());

  int i = resource_path.str().size()-1;
  for(; i >= 0; i--)
  {
    if(resource_path.str()[i] == '/')
      break;
  }

  std::string mesh_name = resource_path.str().substr(i+1);
  mesh_names_.insert(std::pair<int, std::string>(next_mesh_id_, resource_path.str()));
  next_mesh_id_++;

  select_mesh_->addItem(QString(mesh_name.c_str()), QVariant(next_mesh_id_-1));

  visualization_msgs::Marker marker;
  marker.header.frame_id = node_.getGlobalFrame();
  marker.header.stamp = ros::Time();
  marker.ns = "demonstration_visualizer";
  marker.id = next_mesh_id_-1;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
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
 
  node_.publishVisualizationMarker(marker, true);
}

void DemonstrationVisualizer::deleteMesh()
{
  if(selected_mesh_ == -1)
  {
    ROS_INFO("No marker selected.");
    return;
  }

  ROS_INFO("Deleting mesh %d.", selected_mesh_);
  
  if(!node_.removeInteractiveMarker("mesh_marker"))
  {
    ROS_ERROR("Failed to remove interactive marker!");
  }

  visualization_msgs::Marker marker;
  marker.header.frame_id = node_.getGlobalFrame();
  marker.header.stamp = ros::Time();
  marker.ns = "demonstration_visualizer";
  marker.id = selected_mesh_;
  marker.action = visualization_msgs::Marker::DELETE;

  node_.publishVisualizationMarker(marker);

  select_mesh_->removeItem(select_mesh_->findData(QVariant(selected_mesh_)));
  std::map<int, std::string>::iterator it = mesh_names_.find(selected_mesh_);
  if(it != mesh_names_.end())
    mesh_names_.erase(mesh_names_.find(selected_mesh_));

  if(mesh_names_.size() == 0)
    selected_mesh_ = -1;
}

void DemonstrationVisualizer::selectMesh(int mesh_index)
{
  mesh_index -= 1;
  if(mesh_index >= 0)
  {
    ROS_INFO("Selected mesh %d.", (int)select_mesh_->itemData(mesh_index+1).value<int>());
    selected_mesh_ = select_mesh_->itemData(mesh_index+1).value<int>();
  }
}

void DemonstrationVisualizer::interactiveMarkerMoved(
  const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  // Just redraw the marker at the specified pose.
  visualization_msgs::Marker marker;
  marker.header.frame_id = node_.getGlobalFrame();
  marker.header.stamp = ros::Time();
  marker.ns = "demonstration_visualizer";
  marker.id = selected_mesh_;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = feedback->pose;
  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;
  marker.color.a = 0.0;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.mesh_resource = mesh_names_[selected_mesh_];
  marker.mesh_use_embedded_materials = true;
 
  ROS_INFO_STREAM("Moving mesh " << mesh_names_[selected_mesh_] << " with id "
		  << selected_mesh_ << " to pose (" << feedback->pose.position.x
		  << ", " << feedback->pose.position.y << ", " 
		  << feedback->pose.position.z << ")");

  node_.publishVisualizationMarker(marker);  
}
