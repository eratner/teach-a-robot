#include "demonstration_visualizer/demonstration_visualizer.h"

#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QMessageBox>
#include <QDoubleSpinBox>
#include <QPixmap>
#include <QCheckBox>
#include <QInputDialog>
#include <QLineEdit>

DemonstrationVisualizer::DemonstrationVisualizer(int argc, char **argv, QWidget *parent)
 : QWidget(parent), node_(argc, argv)
{
  setWindowTitle("Demonstration Visualizer");

  resize(800, 600);

  setFocusPolicy(Qt::StrongFocus);

  // Initialize Rviz visualization manager and render panel.
  render_panel_ = new rviz::RenderPanel();
  render_panel_->installEventFilter(this);
  visualization_manager_ = new rviz::VisualizationManager(render_panel_);

  render_panel_->initialize(visualization_manager_->getSceneManager(), visualization_manager_);
  visualization_manager_->initialize();
  visualization_manager_->startUpdate();

  // Initialize the window layout.
  QPushButton *toggle_grid = new QPushButton("Toggle Grid", this);
  QPushButton *reset_robot = new QPushButton("Reset Robot", this);

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

  // Set camera (view) properties.
  rviz::ViewManager *view_manager = visualization_manager_->getViewManager();
  view_manager->getCurrent()->subProp("Target Frame")->setValue("/map");

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

  // Change robot speed control panel.
  QHBoxLayout *linear_speed_panel = new QHBoxLayout();
  QLabel *linear_speed_label = new QLabel("Linear Speed (m/s): ");
  linear_speed_panel->addWidget(linear_speed_label);
  QDoubleSpinBox *linear_speed_control = new QDoubleSpinBox();
  linear_speed_panel->addWidget(linear_speed_control);
  linear_speed_control->setSingleStep(0.1);
  linear_speed_control->setValue(0.2);

  QHBoxLayout *angular_speed_panel = new QHBoxLayout();
  QLabel *angular_speed_label = new QLabel("Angular Speed (rad/s): ");
  angular_speed_panel->addWidget(angular_speed_label);
  QDoubleSpinBox *angular_speed_control = new QDoubleSpinBox();
  angular_speed_panel->addWidget(angular_speed_control);
  angular_speed_control->setSingleStep(0.1);
  angular_speed_control->setValue(0.2);

  // Controls for managing the current task.
  QPushButton *add_goal = new QPushButton("Add Goal to Task");
  QHBoxLayout *task_panel = new QHBoxLayout();
  QPushButton *load_task = new QPushButton("Load Task");
  task_panel->addWidget(load_task);
  QPushButton *save_task = new QPushButton("Save Task");
  task_panel->addWidget(save_task);
  QCheckBox *edit_goals = new QCheckBox("Edit Goals Mode");
  edit_goals->setCheckState(Qt::Checked);

  QVBoxLayout *controls_layout = new QVBoxLayout();
  controls_layout->addWidget(toggle_grid);
  controls_layout->addWidget(reset_robot);
  controls_layout->addLayout(recording_controls);
  controls_layout->addLayout(replay_controls);
  controls_layout->addLayout(mesh_controls);
  controls_layout->addLayout(scene_controls);
  controls_layout->addWidget(select_mesh_);
  controls_layout->addWidget(select_tool);
  controls_layout->addLayout(linear_speed_panel);
  controls_layout->addLayout(angular_speed_panel);
  controls_layout->addWidget(add_goal);
  controls_layout->addLayout(task_panel);
  controls_layout->addWidget(edit_goals);

  recording_icon_ = new QLabel();
  //recording_icon_->setPixmap(QPixmap("/home/eratner/Desktop/recording.png"));
  recording_icon_->setAlignment(Qt::AlignLeft);
  recording_icon_->hide();

  QGridLayout *window_layout = new QGridLayout();
  window_layout->addLayout(controls_layout, 0, 0, 1, 1);
  window_layout->addWidget(render_panel_, 0, 1, 2, 3);
  window_layout->addWidget(recording_icon_, 1, 0, 1, 1, Qt::AlignBottom);

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
  visualization_marker_->subProp("Marker Topic")->setValue("/dviz/visualization_marker");
  ROS_ASSERT(visualization_marker_ != NULL);

  // Create an interactive markers display for moving meshes around the scene.
  mesh_interactive_markers_ = visualization_manager_->createDisplay("rviz/InteractiveMarkers",
								    "Mesh Interactive Markers",
								    true);
  ROS_ASSERT(mesh_interactive_markers_ != NULL);
  mesh_interactive_markers_->subProp("Update Topic")->setValue("/mesh_marker/update");

  // Connect signals to appropriate slots.
  connect(toggle_grid, SIGNAL(clicked()), this, SLOT(toggleGrid()));
  connect(reset_robot, SIGNAL(clicked()), this, SLOT(resetRobot()));
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
  connect(linear_speed_control, SIGNAL(valueChanged(double)), this, SLOT(setLinearSpeed(double)));
  connect(angular_speed_control, SIGNAL(valueChanged(double)), this, SLOT(setAngularSpeed(double)));
  connect(add_goal, SIGNAL(clicked()), this, SLOT(addTaskGoal()));
  connect(load_task, SIGNAL(clicked()), this, SLOT(loadTask()));
  connect(save_task, SIGNAL(clicked()), this, SLOT(saveTask()));
  connect(edit_goals, SIGNAL(stateChanged(int)), this, SLOT(setEditGoalsMode(int)));
  connect(&node_, SIGNAL(goalComplete(int)), this, SLOT(notifyGoalComplete(int)));

  // Close window when ROS shuts down.
  connect(&node_, SIGNAL(rosShutdown()), this, SLOT(close()));

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

void DemonstrationVisualizer::keyPressEvent(QKeyEvent *event)
{
  switch(event->key())
  {
  case Qt::Key_Up:
    node_.processKeyEvent(Qt::Key_Up, QEvent::KeyPress);
    break;
  case Qt::Key_Down:
    node_.processKeyEvent(Qt::Key_Down, QEvent::KeyPress);
    break;
  default:
    QWidget::keyPressEvent(event);
    break;
  }
}

void DemonstrationVisualizer::keyReleaseEvent(QKeyEvent *event)
{
  switch(event->key())
  {
  case Qt::Key_Up:
    node_.processKeyEvent(Qt::Key_Up, QEvent::KeyRelease);
    break;
  case Qt::Key_Down:
    node_.processKeyEvent(Qt::Key_Down, QEvent::KeyRelease);
    break;
  default:
    QWidget::keyReleaseEvent(event);
    break;
  }
}

bool DemonstrationVisualizer::eventFilter(QObject *obj, QEvent *event)
{
  if(event->type() == QEvent::KeyPress)
  {
    QKeyEvent *key_event = static_cast<QKeyEvent *>(event);
    keyPressEvent(key_event);
    return true;
  }
  else if(event->type() == QEvent::KeyRelease)
  {
    QKeyEvent *key_event = static_cast<QKeyEvent *>(event);
    keyReleaseEvent(key_event);
    return true;
  }
  else
  {
    return QObject::eventFilter(obj, event);
  }
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

void DemonstrationVisualizer::resetRobot()
{
  node_.resetRobot();
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

  recording_icon_->show();

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

  recording_icon_->hide();

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
  std_srvs::Empty empty;
  if(!node_.endReplay(empty))
  {
    ROS_ERROR("[DViz] Failed to end replay!");
  }
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

  node_.getSceneManager()->addMesh(marker);
 
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

  node_.getSceneManager()->removeMesh(selected_mesh_);

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
      node_.getSceneManager()->loadScene(filename.toStdString());
      
      // Re-publish each mesh marker.
      node_.clearInteractiveMarkers();
      std::vector<visualization_msgs::Marker> meshes = node_.getSceneManager()->getMeshes();
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

  node_.getSceneManager()->saveScene(filename.toStdString());
}

void DemonstrationVisualizer::loadTask()
{
  QString filename = QFileDialog::getOpenFileName(this, 
						  tr("Open Demonstration Task File"),
						  "/home",
						  tr("Demonstration Task Files (*.xml)"));

  if(filename.isEmpty())
  {
    ROS_INFO("No file selected.");
    return;
  }  

  switch(QMessageBox::warning(this,
			      "Clear Current Demonstration Task?",
			      "Loading a new scene will clear the current demonstration task."
			      " Are you sure you wish to do this?",
			      QMessageBox::Yes, QMessageBox::No))
  {
  case QMessageBox::Yes:
    {
      // Load the task into the demonstration scene manager.
      node_.getSceneManager()->loadTask(filename.toStdString());
      break;
    }
  case QMessageBox::No:
    break;
  default:
    ROS_ERROR("[DViz] An error has occured in loading the task!");
    break;
  }
}

void DemonstrationVisualizer::saveTask()
{
  QString filename = QFileDialog::getSaveFileName(this, 
						  tr("Save Demonstration Task File"),
						  "/home",
						  tr("Demonstration Task Files (*.xml)"));

  if(filename.isEmpty())
  {
    ROS_INFO("No file selected.");
    return;
  }

  node_.getSceneManager()->saveTask(filename.toStdString());
}

void DemonstrationVisualizer::setEditGoalsMode(int mode)
{
  switch(mode)
  {
  case Qt::Unchecked:
    node_.setEditGoalsMode(false);
    node_.getSceneManager()->setGoalsChanged(true);
    break;
  case Qt::Checked:
    node_.setEditGoalsMode(true);
    node_.getSceneManager()->setGoalsChanged(true);
    break;
  default:
    ROS_ERROR("[DViz] Invalid edit goals mode!");
    break;
  }
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

void DemonstrationVisualizer::setLinearSpeed(double linear)
{
  node_.setRobotSpeed(linear, 0);
}

void DemonstrationVisualizer::setAngularSpeed(double angular)
{
  node_.setRobotSpeed(0, angular);
}

void DemonstrationVisualizer::addTaskGoal()
{
  bool ok;
  QString description = QInputDialog::getText(this,
					      tr("Add a Goal Description"),
					      tr("Description:"),
					      QLineEdit::Normal,
					      "",
					      &ok);

  std::string desc = "";
  if(ok)
    desc = description.toStdString();

  node_.getSceneManager()->addGoal(geometry_msgs::Pose(), desc);
}

void DemonstrationVisualizer::notifyGoalComplete(int goal_number)
{
  QMessageBox box;

  std::stringstream text;
  text << "Goal " << goal_number << " complete!";

  if(goal_number+1 >= node_.getSceneManager()->getNumGoals())
  {
    text << " All goals complete!";
  }
  else
  {
    text << "Next goal:\n" << node_.getSceneManager()->getGoalDescription(goal_number+1);
  }
  
  box.setText(QString(text.str().c_str()));

  box.exec();
}
