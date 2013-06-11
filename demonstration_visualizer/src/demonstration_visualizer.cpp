#include "demonstration_visualizer/demonstration_visualizer.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QDoubleSpinBox>
#include <QLabel>

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

  // Load/delete mesh control panel.
  QHBoxLayout *mesh_controls = new QHBoxLayout();
  QPushButton *load_mesh = new QPushButton("Load Mesh");
  mesh_controls->addWidget(load_mesh);
  QPushButton *delete_mesh = new QPushButton("Delete Mesh");
  mesh_controls->addWidget(delete_mesh);

  // Load/save scene control panel.
  QHBoxLayout *scene_controls = new QHBoxLayout();
  QPushButton *load_scene = new QPushButton("Load Scene");
  scene_controls->addWidget(load_scene);
  QPushButton *save_scene = new QPushButton("Save Scene");
  scene_controls->addWidget(save_scene);

  // Change robot velocity control panel.
  QHBoxLayout *linear_velocity_panel = new QHBoxLayout();
  QLabel *linear_velocity_label = new QLabel("Linear Velocity (m/s): ");
  linear_velocity_panel->addWidget(linear_velocity_label);
  QDoubleSpinBox *linear_velocity_control = new QDoubleSpinBox();
  linear_velocity_panel->addWidget(linear_velocity_control);
  linear_velocity_control->setSingleStep(0.1);
  linear_velocity_control->setValue(0.2);

  QHBoxLayout *angular_velocity_panel = new QHBoxLayout();
  QLabel *angular_velocity_label = new QLabel("Angular Velocity (rad/s): ");
  angular_velocity_panel->addWidget(angular_velocity_label);
  QDoubleSpinBox *angular_velocity_control = new QDoubleSpinBox();
  angular_velocity_panel->addWidget(angular_velocity_control);
  angular_velocity_control->setSingleStep(0.1);
  angular_velocity_control->setValue(0.2);

  QVBoxLayout *controls_layout = new QVBoxLayout();
  controls_layout->addWidget(toggle_grid);
  controls_layout->addLayout(recording_controls);
  controls_layout->addLayout(replay_controls);
  controls_layout->addLayout(mesh_controls);
  controls_layout->addLayout(scene_controls);
  controls_layout->addWidget(select_mesh_);
  controls_layout->addWidget(select_tool);
  controls_layout->addLayout(linear_velocity_panel);
  controls_layout->addLayout(angular_velocity_panel);

  QGridLayout *window_layout = new QGridLayout();
  window_layout->addLayout(controls_layout, 0, 0, 1, 1);
  window_layout->addWidget(render_panel_, 0, 1, 2, 3);

  // Create and display a grid.
  grid_ = visualization_manager_->createDisplay("rviz/Grid", "Grid", true);
  ROS_ASSERT(grid_ != NULL);

  // Create a robot model display.
  robot_model_ = visualization_manager_->createDisplay("rviz/MarkerArray", "Robot", true);
  ROS_ASSERT(robot_model_ != NULL);
  robot_model_->subProp("Marker Topic")->setValue("/visualization_marker_array");
  
  // Create an interactive markers display for controlling the robot. 
  robot_interactive_markers_ = visualization_manager_->createDisplay("rviz/InteractiveMarkers", 
								     "PR2 Interactive Markers", 
								     true);
  ROS_ASSERT(robot_interactive_markers_ != NULL);
  robot_interactive_markers_->subProp("Update Topic")->setValue("/simple_sim_marker/update");

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
  connect(load_scene, SIGNAL(clicked()), this, SLOT(loadScene()));
  connect(save_scene, SIGNAL(clicked()), this, SLOT(saveScene()));
  connect(&node_, 
	  SIGNAL(interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)),
	  this,
	  SLOT(interactiveMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &)));
  connect(linear_velocity_control, SIGNAL(valueChanged(double)), this, SLOT(setLinearVelocity(double)));
  connect(angular_velocity_control, SIGNAL(valueChanged(double)), this, SLOT(setAngularVelocity(double)));

  next_mesh_id_ = 3;
  selected_mesh_ = -1;

  setLayout(window_layout);

  node_.resetRobot();
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

  // Spawn the mesh at the origin.
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = node_.getGlobalFrame();
  pose_stamped.header.stamp = ros::Time();
  pose_stamped.pose.position.x = 0;
  pose_stamped.pose.position.y = 0;
  pose_stamped.pose.position.z = 0;
  pose_stamped.pose.orientation.x = 0.0;
  pose_stamped.pose.orientation.y = 0.0;
  pose_stamped.pose.orientation.z = 0.0;
  pose_stamped.pose.orientation.w = 1.0;  
  geometry_msgs::Vector3 scale;
  scale.x = scale.y = scale.z = 1;

  visualization_msgs::Marker marker = makeMeshMarker(resource_path.str(),
						     "demonstration_visualizer",
						     next_mesh_id_-1,
						     pose_stamped,
						     scale,
						     0.0,
						     true);

  demonstration_scene_manager_.addMesh(marker);
 
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

  std::stringstream int_marker_name;
  int_marker_name << "mesh_marker_" << selected_mesh_;
  
  if(!node_.removeInteractiveMarker(int_marker_name.str().c_str()))
  {
    ROS_ERROR("Failed to remove interactive marker!");
  }

  demonstration_scene_manager_.removeMesh(selected_mesh_);

  // @todo it would be cleaner to let the demonstration scene manager handle this.
  select_mesh_->removeItem(select_mesh_->findData(QVariant(selected_mesh_)));
  std::map<int, std::string>::iterator it = mesh_names_.find(selected_mesh_);
  if(it != mesh_names_.end())
    mesh_names_.erase(mesh_names_.find(selected_mesh_));

  if(mesh_names_.size() == 0)
    selected_mesh_ = -1;
}

void DemonstrationVisualizer::loadScene()
{
  QString filename = QFileDialog::getOpenFileName(this, 
						  tr("Open Demonstration Scene File"),
						  "/home",
						  tr("Demonstration Scene Files (*.xml)"));

  if(filename.isEmpty())
  {
    ROS_INFO("No file selected.");
    return;
  }  

  switch(QMessageBox::warning(this,
			      "Clear Current Demonstration Scene?",
			      "Loading a new scene will clear the current demonstration scene."
			      " Are you sure you wish to do this?",
			      QMessageBox::Yes, QMessageBox::No))
  {
  case QMessageBox::Yes:
    {
      // Load the scene into the demonstration scene manager.
      demonstration_scene_manager_.loadScene(filename.toStdString());
      
      // Re-publish each mesh marker.
      node_.clearInteractiveMarkers();
      std::vector<visualization_msgs::Marker> meshes = demonstration_scene_manager_.getMeshes();
      std::vector<visualization_msgs::Marker>::iterator it;
      int max_mesh_id = -1;
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	if(it->id > max_mesh_id)
	  max_mesh_id = it->id;

	it->header.frame_id = node_.getGlobalFrame();
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::ADD;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->mesh_use_embedded_materials = true;

	int i = it->mesh_resource.size()-1;
	for(; i >= 0; i--)
	{
	  if(it->mesh_resource[i] == '/')
	    break;
	}

	std::string mesh_name = it->mesh_resource.substr(i+1);
	mesh_names_.insert(std::pair<int, std::string>(it->id, mesh_name));

	select_mesh_->addItem(QString(it->mesh_resource.c_str()), QVariant(it->id));

	ROS_INFO_STREAM("Adding mesh " << it->id << " ns = "
			<< it->ns << " mesh_resource = " 
			<< it->mesh_resource << " pos = ("
			<< it->pose.position.x << ", " 
			<< it->pose.position.y << ", "
			<< it->pose.position.z << ").");

	node_.publishVisualizationMarker(*it, true);
      }

      next_mesh_id_ = max_mesh_id;

      break;
    }
  case QMessageBox::No:
    break;
  default:
    ROS_ERROR("An error has occured in loading the scene!");
    break;
  }
}

void DemonstrationVisualizer::saveScene()
{
  QString filename = QFileDialog::getSaveFileName(this, 
						  tr("Save Demonstration Scene File"),
						  "/home",
						  tr("Demonstration Scene Files (*.xml)"));

  if(filename.isEmpty())
  {
    ROS_INFO("No file selected.");
    return;
  }

  demonstration_scene_manager_.saveScene(filename.toStdString());
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

void DemonstrationVisualizer::interactiveMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  int i;
  for(i = feedback->marker_name.size()-1; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  if(!demonstration_scene_manager_.updateMeshPose(atoi(feedback->marker_name.substr(i+1).c_str()),
						  feedback->pose))
  {
    ROS_ERROR("Demonstration scene manager failed to update pose!");
  }
}

void DemonstrationVisualizer::setLinearVelocity(double lin_vel)
{
  node_.setRobotVelocity(lin_vel, 0);
}

void DemonstrationVisualizer::setAngularVelocity(double ang_vel)
{
  node_.setRobotVelocity(0, ang_vel);
}
