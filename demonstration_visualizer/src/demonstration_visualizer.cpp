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
#include <QSlider>
#include <QGroupBox>
#include <QSignalMapper>

#include <ros/package.h>

DemonstrationVisualizer::DemonstrationVisualizer(int argc, char **argv, QWidget *parent)
 : QWidget(parent), node_(argc, argv)
{
  setWindowTitle("Demonstration Visualizer");

  resize(1000, 800);

  setFocusPolicy(Qt::StrongFocus);

  qRegisterMetaType<geometry_msgs::Pose>("geometry_msgs::Pose");

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

  // Scale meshes.
  QHBoxLayout *scale_mesh_panel = new QHBoxLayout();
  QLabel *scale_mesh_label = new QLabel("Scale Mesh: ");
  scale_mesh_panel->addWidget(scale_mesh_label);
  QSlider *scale_mesh = new QSlider(Qt::Horizontal);
  scale_mesh->setMinimum(1);
  scale_mesh->setMaximum(200);
  scale_mesh->setValue(100);
  scale_mesh_panel->addWidget(scale_mesh);

  rviz::ToolManager *tool_manager = visualization_manager_->getToolManager();
  ROS_INFO("There are %d tools loaded:", tool_manager->numTools());
  for(int i = 0; i < tool_manager->numTools(); ++i)
  {
    ROS_INFO("%d. %s", i, tool_manager->getTool(i)->getClassId().toLocal8Bit().data());
    select_tool->addItem(tool_manager->getTool(i)->getClassId());
  }

  // Set camera (view) properties.
  rviz::ViewManager *view_manager = visualization_manager_->getViewManager();
  view_manager->getCurrent()->subProp("Target Frame")->setValue("/base_footprint");

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
  QHBoxLayout *edit_mode_panel = new QHBoxLayout();
  QCheckBox *edit_goals = new QCheckBox("Edit Goals Mode");
  edit_goals->setCheckState(Qt::Checked);
  edit_mode_panel->addWidget(edit_goals);
  QCheckBox *edit_scene = new QCheckBox("Edit Scene Mode");
  edit_scene->setCheckState(Qt::Checked);
  edit_mode_panel->addWidget(edit_scene);
  QLabel *goals_label = new QLabel("Goals:");
  goals_list_ = new QListWidget();
  goals_list_->setSelectionRectVisible(false);
  goals_list_->setWrapping(true);
  goals_list_->addItem("No task loaded.");

  QVBoxLayout *controls_layout = new QVBoxLayout();
  controls_layout->addWidget(toggle_grid);
  controls_layout->addWidget(reset_robot);
  controls_layout->addLayout(recording_controls);
  controls_layout->addLayout(replay_controls);
  controls_layout->addLayout(mesh_controls);
  controls_layout->addLayout(scene_controls);
  controls_layout->addWidget(select_mesh_);
  controls_layout->addLayout(scale_mesh_panel);
  controls_layout->addWidget(select_tool);
  controls_layout->addLayout(linear_speed_panel);
  controls_layout->addLayout(angular_speed_panel);
  controls_layout->addWidget(add_goal);
  controls_layout->addLayout(task_panel);
  controls_layout->addLayout(edit_mode_panel);
  controls_layout->addWidget(goals_label);

  QHBoxLayout *status_icons = new QHBoxLayout();
  recording_icon_ = new QLabel();
  recording_icon_->setPixmap(QPixmap(":/images/recording.png"));
  recording_icon_->setAlignment(Qt::AlignLeft);
  recording_icon_->hide();
  status_icons->addWidget(recording_icon_);
  replaying_icon_ = new QLabel();
  replaying_icon_->setPixmap(QPixmap(":/images/replaying.png"));
  replaying_icon_->setAlignment(Qt::AlignLeft);
  replaying_icon_->hide();
  status_icons->addWidget(replaying_icon_);

  QGridLayout *l_window_layout = new QGridLayout();
  l_window_layout->addLayout(controls_layout, 0, 0, 1, 1);
  l_window_layout->addLayout(status_icons, 1, 0, 1, 1, Qt::AlignBottom);

  // The basic control panel.
  QVBoxLayout *basic_layout = new QVBoxLayout();
  start_button_ = new QPushButton("Start!");
  basic_layout->addWidget(start_button_);
  end_button_ = new QPushButton("End");
  basic_layout->addWidget(end_button_);
  QHBoxLayout *play_pause_layout = new QHBoxLayout();
  QPushButton *play = new QPushButton("Play");
  QPushButton *pause = new QPushButton("Pause");
  play_pause_layout->addWidget(play);
  play_pause_layout->addWidget(pause);
  basic_layout->addLayout(play_pause_layout);

  QGroupBox *camera_group = new QGroupBox("Camera Settings");
  QHBoxLayout *camera_controls = new QHBoxLayout();
  QPushButton *orbit_camera = new QPushButton("Orbit");
  camera_buttons_.push_back(orbit_camera);
  QPushButton *fps_camera = new QPushButton("FPS");
  camera_buttons_.push_back(fps_camera);
  QPushButton *top_down_camera = new QPushButton ("Top Down");
  camera_buttons_.push_back(top_down_camera);
  QPushButton *auto_camera = new QPushButton("Auto");
  camera_buttons_.push_back(auto_camera);

  camera_controls->addWidget(orbit_camera);
  camera_controls->addWidget(fps_camera);
  camera_controls->addWidget(top_down_camera);
  camera_controls->addWidget(auto_camera);
  camera_group->setLayout(camera_controls);

  basic_layout->addWidget(camera_group);

  QGroupBox *controls_group = new QGroupBox("Controls");
  QVBoxLayout *user_controls_layout = new QVBoxLayout();
  z_mode_button_ = new QPushButton("Enable Z-Mode");
  user_controls_layout->addWidget(z_mode_button_);

  QHBoxLayout *fps_x_offset_layout = new QHBoxLayout();
  QLabel *fps_x_offset_label = new QLabel("FPS x-offset: ");
  fps_x_offset_layout->addWidget(fps_x_offset_label);
  QSlider *fps_x_offset = new QSlider(Qt::Horizontal);
  fps_x_offset->setMinimum(0);
  fps_x_offset->setMaximum(500);
  fps_x_offset->setValue(0);
  fps_x_offset_layout->addWidget(fps_x_offset);
  user_controls_layout->addLayout(fps_x_offset_layout);

  QHBoxLayout *fps_z_offset_layout = new QHBoxLayout();
  QLabel *fps_z_offset_label = new QLabel("FPS z-offset: ");
  fps_z_offset_layout->addWidget(fps_z_offset_label);
  QSlider *fps_z_offset = new QSlider(Qt::Horizontal);
  fps_z_offset->setMinimum(0);
  fps_z_offset->setMaximum(500);
  fps_z_offset->setValue(0);
  fps_z_offset_layout->addWidget(fps_z_offset);
  user_controls_layout->addLayout(fps_z_offset_layout);

  controls_group->setLayout(user_controls_layout);

  basic_layout->addWidget(controls_group);

  basic_layout->addWidget(goals_list_);

  basic_ = new QWidget();
  basic_->setLayout(basic_layout);
  advanced_ = new QWidget();
  advanced_->setLayout(l_window_layout);

  // The controls panel.
  controls_info_ = new QWidget();
  QLabel *controls_info = new QLabel("<strong>Controls</strong>:"
				     "<ul>"
				     "<li>Left-click on the base, and drag <br />to the desired position in the <i>xy</i>-plane.</li>"
				     "<li>Left-click on the gripper, and move <br />to the desired position in the <i>x</i> and <i>y</i> directions.</li>"
				     "<li>Press the <b>Up</b> arrow key to move the gripper up.</li>"
				     "<li>Press the <b>Down</b> arrow key to move the gripper down.</li>"
				     "<hr />"
				     "<li>Press and hold the <b>Z</b> key, and left-click <br />on the gripper to move in the <i>z</i> direction.</li>"
				     "<hr />"
				     "<li>Press the <b>R</b> key to reset the gripper marker.</li>"
				     "<hr />"
				     "<li>In FPS mode, use the <b>W</b>, <b>A</b>, <b>S</b>, and <b>D</b> keys <br />to move the base forward, left, backward, <br />and right, respectively.</li>"
				     "<li>In FPS mode, yaw the camera to rotate the base.</li>"
				     "</ul>");
  controls_info->setAlignment(Qt::AlignTop);
  QVBoxLayout *controls_info_layout = new QVBoxLayout();
  controls_info_layout->addWidget(controls_info, Qt::AlignTop);
  controls_info_->setLayout(controls_info_layout);

  QTabWidget *tabs = new QTabWidget();
  tabs->addTab(basic_, tr("Basic"));
  tabs->addTab(advanced_, tr("Advanced"));
  tabs->addTab(controls_info_, tr("Controls"));

  QHBoxLayout *window_layout = new QHBoxLayout();
  window_layout->addWidget(tabs, 1);
  window_layout->addWidget(render_panel_, 4);

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
  visualization_marker_->subProp("Marker Topic")->setValue("/visualization_marker");
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
  connect(edit_scene, SIGNAL(stateChanged(int)), this, SLOT(setEditSceneMode(int)));
  connect(&node_, SIGNAL(goalComplete(int)), this, SLOT(notifyGoalComplete(int)));
  connect(goals_list_, 
	  SIGNAL(itemDoubleClicked(QListWidgetItem *)), this, 
	  SLOT(editGoalDescription(QListWidgetItem *))
	  );
  connect(tabs, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));
  connect(start_button_, SIGNAL(clicked()), this, SLOT(startBasicMode()));
  connect(end_button_, SIGNAL(clicked()), this, SLOT(endBasicMode()));
  connect(&node_, SIGNAL(focusCameraTo(float, float, float)), this, SLOT(focusCameraTo(float, float, float)));
  connect(scale_mesh, SIGNAL(valueChanged(int)), this, SLOT(scaleMesh(int)));
  connect(play, SIGNAL(clicked()), this, SLOT(playSimulator()));
  connect(pause, SIGNAL(clicked()), this, SLOT(pauseSimulator()));

  QSignalMapper *camera_signal_mapper = new QSignalMapper(this);
  connect(camera_signal_mapper, SIGNAL(mapped(int)), this, SLOT(changeCameraMode(int)));

  connect(orbit_camera, SIGNAL(clicked()), camera_signal_mapper, SLOT(map()));
  camera_signal_mapper->setMapping(orbit_camera, (int)ORBIT);
  connect(fps_camera, SIGNAL(clicked()), camera_signal_mapper, SLOT(map()));
  camera_signal_mapper->setMapping(fps_camera, (int)FPS);
  connect(top_down_camera, SIGNAL(clicked()), camera_signal_mapper, SLOT(map()));
  camera_signal_mapper->setMapping(top_down_camera, (int)TOP_DOWN);
  connect(auto_camera, SIGNAL(clicked()), camera_signal_mapper, SLOT(map()));
  camera_signal_mapper->setMapping(auto_camera, (int)AUTO);

  // Close window when ROS shuts down.
  connect(&node_, SIGNAL(rosShutdown()), this, SLOT(close()));

  connect(&node_, SIGNAL(updateCamera(const geometry_msgs::Pose &, const geometry_msgs::Pose &)), this, 
	  SLOT(updateCamera(const geometry_msgs::Pose &, const geometry_msgs::Pose &)));

  connect(z_mode_button_, SIGNAL(clicked()), this, SLOT(toggleZMode()));
  connect(fps_x_offset, SIGNAL(valueChanged(int)), this, SLOT(setFPSXOffset(int)));
  connect(fps_z_offset, SIGNAL(valueChanged(int)), this, SLOT(setFPSZOffset(int)));

  next_mesh_id_ = 3;
  selected_mesh_ = -1;
  previous_camera_mode_ = camera_mode_ = ORBIT;
  z_mode_ = false;
  x_fps_offset_ = 0;
  z_fps_offset_ = 0;

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
  if(event->isAutoRepeat())
    event->ignore();
  else
  {
    node_.processKeyEvent(event->key(), QEvent::KeyPress);
    QWidget::keyPressEvent(event);
  }
}

void DemonstrationVisualizer::keyReleaseEvent(QKeyEvent *event)
{
  if(event->isAutoRepeat())
    event->ignore();
  else
  {
    node_.processKeyEvent(event->key(), QEvent::KeyRelease);
    QWidget::keyReleaseEvent(event);
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
  else if(event->type() == QEvent::Wheel)
  {
    if(camera_mode_ == FPS || camera_mode_ == TOP_DOWN)
      return true;
    else if(camera_mode_ == ORBIT || camera_mode_ == AUTO)
    {
      QWheelEvent *wheel_event = static_cast<QWheelEvent *>(event);
      int degrees = wheel_event->delta() / 8;
      int steps = -degrees / 15;

      double current_distance = 
	visualization_manager_->getViewManager()->getCurrent()->subProp("Distance")->getValue().toDouble();

      if(steps > 0 && current_distance < 8.0)
      {
	current_distance += steps;
	visualization_manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue(current_distance);
      }
      else if(steps < 0 && current_distance > 3.0)
      {
	current_distance += steps;
	visualization_manager_->getViewManager()->getCurrent()->subProp("Distance")->setValue(current_distance);
      }

      return true;
    }
    else
      return QObject::eventFilter(obj, event);
  }
  // else if(event->type() == QEvent::MouseButtonPress ||
  // 	  event->type() == QEvent::MouseButtonRelease)
  // {
  //   // @todo is there a way to filter events related to camera movements, but NOT
  //   // related to moving the interactive markers?
  //   if(camera_mode_ == TOP_DOWN && user_demo_.started_ == false)
  //     return true;
  //   else
  //     return QObject::eventFilter(obj, event);
  // }
  else if(event->type() == QEvent::MouseMove)
  {
    if(camera_mode_ == TOP_DOWN)
    {
      return true;
    }
    else
      return QObject::eventFilter(obj, event);
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

void DemonstrationVisualizer::resetTask()
{
  user_demo_.goals_completed_ = 0;

  node_.setCurrentGoal(0);

  // Reset the goal list.
  for(int i = 0; i < node_.getSceneManager()->getNumGoals(); ++i)
  {
    QFont font = goals_list_->item(i)->font();
    font.setBold(i == 0 ? true : false);
    font.setStrikeOut(false);
    goals_list_->item(i)->setFont(font);     
  }

  node_.getSceneManager()->setGoalsChanged(true);
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

  pr2_simple_simulator::FilePath srv;
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

  pr2_simple_simulator::FilePath srv;
  srv.request.file_path = filename.toStdString();
  if(!node_.beginReplay(srv))
  {
    ROS_ERROR("Failed to call service /motion_recorder/begin_replay.");
    return;
  }

  std_srvs::Empty empty;
  if(!node_.playSimulator(empty))
  {
    ROS_ERROR("Failed to call service to play simulator.");
  }

  replaying_icon_->show();

  ROS_INFO("Replaying from file %s", filename.toLocal8Bit().data());
}

void DemonstrationVisualizer::endReplay()
{
  std_srvs::Empty empty;
  if(!node_.endReplay(empty))
  {
    ROS_ERROR("[DViz] Failed to end replay!");
  }

  replaying_icon_->hide();
}

void DemonstrationVisualizer::loadMesh()
{
  std::string package_path = ros::package::getPath("demonstration_visualizer");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package demonstration_visualizer!");
    return;
  }

  QString filename = QFileDialog::getOpenFileName(this, 
						  tr("Open Mesh File"),
						  QString(package_path.c_str()),
						  tr("Mesh Files (*.dae *.stl *.mesh)"));

  if(filename.isEmpty())
  {
    ROS_INFO("No file selected.");
    return;
  }

  std::size_t found = filename.toStdString().find("demonstration_visualizer");
  if(found == std::string::npos)
  {
    ROS_ERROR("[DViz] Please load a mesh that is within a subdirectory of the demonstration_visualizer pacakge.");
    return;
  }

  std::stringstream resource_path;
  resource_path << "package://" << filename.toStdString().substr(found);
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
    ROS_INFO("[DViz] No marker selected.");
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

void DemonstrationVisualizer::setEditSceneMode(int mode)
{
  switch(mode)
  {
  case Qt::Unchecked:
    {
      std::vector<visualization_msgs::Marker> meshes = node_.getSceneManager()->getMeshes();
      std::vector<visualization_msgs::Marker>::iterator it;
      // For each mesh, first remove all the interactive markers from the meshes.
      // Then, re-visualize each marker without an attached interactive marker.
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	std::stringstream int_marker_name;
	int_marker_name << "mesh_marker_" << it->id;

	if(!node_.removeInteractiveMarker(int_marker_name.str().c_str()))
	{
	  ROS_ERROR("[DViz] Failed to remove interactive marker on mesh %d!", it->id);
	}

	it->header.frame_id = "/map";
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::ADD;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->color.r = it->color.g = it->color.b = it->color.a = 0;
	it->mesh_use_embedded_materials = true;

	node_.publishVisualizationMarker(*it, false);
      }
      
      break;
    }
  case Qt::Checked:
    {
      std::vector<visualization_msgs::Marker> meshes = node_.getSceneManager()->getMeshes();
      std::vector<visualization_msgs::Marker>::iterator it;
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	it->header.frame_id = node_.getGlobalFrame();
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::DELETE;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->mesh_use_embedded_materials = true;

	// First remove the old markers.
	node_.publishVisualizationMarker(*it, false);

	// Then add the markers again, but this time with interactive markers.
	it->action = visualization_msgs::Marker::ADD;
	node_.publishVisualizationMarker(*it, true);
      }

      break;
    }
  default:
    ROS_ERROR("[DViz] Invalid edit scene mode!");
    break;
  }
}

void DemonstrationVisualizer::loadScene()
{
  std::string package_path = ros::package::getPath("demonstration_visualizer");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package demonstration_visualizer!");
    return;
  }

  QString filename = QFileDialog::getOpenFileName(this, 
						  tr("Open Demonstration Scene File"),
						  QString(package_path.c_str()),
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

	select_mesh_->addItem(QString(mesh_name.c_str()), QVariant(it->id));

	ROS_INFO_STREAM("Adding mesh " << it->id << " ns = "
			<< it->ns << " mesh_resource = " 
			<< it->mesh_resource << " pos = ("
			<< it->pose.position.x << ", " 
			<< it->pose.position.y << ", "
			<< it->pose.position.z << ").");

	node_.publishVisualizationMarker(*it, true);
      }

      next_mesh_id_ = max_mesh_id+1;

      break;
    }
  case QMessageBox::No:
    break;
  default:
    ROS_ERROR("[DViz] An error has occured in loading the scene!");
    break;
  }
}

void DemonstrationVisualizer::saveScene()
{
  std::string package_path = ros::package::getPath("demonstration_visualizer");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package demonstration_visualizer!");
    return;
  }

  QString filename = QFileDialog::getSaveFileName(this, 
						  tr("Save Demonstration Scene File"),
						  QString(package_path.c_str()),
						  tr("Demonstration Scene Files (*.xml)"));

  if(filename.isEmpty())
  {
    ROS_INFO("[DViz] No file selected.");
    return;
  }

  node_.getSceneManager()->saveScene(filename.toStdString());
}

void DemonstrationVisualizer::loadTask()
{
  std::string package_path = ros::package::getPath("demonstration_visualizer");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package demonstration_visualizer!");
    return;
  }  

  QString filename = QFileDialog::getOpenFileName(this, 
						  tr("Open Demonstration Task File"),
						  QString(package_path.c_str()),
						  tr("Demonstration Task Files (*.xml)"));

  if(filename.isEmpty())
  {
    ROS_INFO("[DViz] No file selected.");
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
      goals_list_->clear();
      std::vector<visualization_msgs::Marker> goals = node_.getSceneManager()->getGoals();
      for(int i = 0; i < node_.getSceneManager()->getNumGoals(); ++i)
      {
	std::stringstream goal_desc;
	goal_desc << "Goal " << i+1 << ": " << node_.getSceneManager()->getGoalDescription(i);
	goals_list_->addItem(QString(goal_desc.str().c_str()));
      }
      
      // Bold the first entry to indicate that is the current goal.
      QFont font = goals_list_->item(0)->font();
      font.setBold(true);
      goals_list_->item(0)->setFont(font);
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
  std::string package_path = ros::package::getPath("demonstration_visualizer");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package demonstration_visualizer!");
    return;
  }

  QString filename = QFileDialog::getSaveFileName(this, 
						  tr("Save Demonstration Task File"),
						  QString(package_path.c_str()),
						  tr("Demonstration Task Files (*.xml)"));

  if(filename.isEmpty())
  {
    ROS_INFO("[DViz] No file selected.");
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
    ROS_INFO("[DViz] Selected mesh %d.", (int)select_mesh_->itemData(mesh_index+1).value<int>());
    selected_mesh_ = select_mesh_->itemData(mesh_index+1).value<int>();
  }
}

void DemonstrationVisualizer::scaleMesh(int value)
{
  if(selected_mesh_ == -1)
  {
    ROS_INFO("[DViz] No marker selected to scale.");
    return;
  }

  std::stringstream int_marker_name;
  int_marker_name << "mesh_marker_" << selected_mesh_;
  
  if(!node_.removeInteractiveMarker(int_marker_name.str().c_str()))
  {
    ROS_ERROR("[DViz] Failed to remove interactive marker!");
  }

  visualization_msgs::Marker mesh = node_.getSceneManager()->getMesh(selected_mesh_);

  double scale_factor = value/100.0;

  ROS_INFO("[DViz] Scaling mesh %d by a factor of %f.", 
	   select_mesh_->itemData(select_mesh_->currentIndex()).value<int>(),
	   scale_factor);

  mesh.scale.x = scale_factor;
  mesh.scale.y = scale_factor;
  mesh.scale.z = scale_factor;

  node_.getSceneManager()->updateMeshScale(selected_mesh_, scale_factor, scale_factor, scale_factor);

  node_.publishVisualizationMarker(mesh, true);
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
  else
  {
    ROS_INFO("[DViz] No task added.");
    return;
  }

  if(node_.getSceneManager()->getNumGoals() == 0)
    goals_list_->clear();

  node_.getSceneManager()->addGoal(geometry_msgs::Pose(), desc);
  std::stringstream goal_desc;
  goal_desc << "Goal " << node_.getSceneManager()->getNumGoals() << ": " << desc;
  goals_list_->addItem(QString(goal_desc.str().c_str()));
}

void DemonstrationVisualizer::editGoalDescription(QListWidgetItem *goal)
{
  int goal_number = goals_list_->row(goal);
  std::string current_desc = node_.getSceneManager()->getGoalDescription(goal_number);

  bool ok;
  QString new_desc = QInputDialog::getText(this,
					   tr("Edit Goal Description:"),
					   tr("Description:"),
					   QLineEdit::Normal,
					   QString(current_desc.c_str()),
					   &ok);
  if(ok)
  {
    node_.getSceneManager()->setGoalDescription(goal_number, new_desc.toStdString());
    std::stringstream goal_desc;
    goal_desc << "Goal " << goal_number+1 << ": " << new_desc.toStdString();
    goal->setData(Qt::DisplayRole, QString(goal_desc.str().c_str()));
  }
}

void DemonstrationVisualizer::notifyGoalComplete(int goal_number)
{
  pauseSimulator();

  // Update the task list.
  QFont font = goals_list_->item(goal_number)->font();
  font.setBold(false);
  font.setStrikeOut(true);
  goals_list_->item(goal_number)->setFont(font);   

  if(goal_number < node_.getSceneManager()->getNumGoals()-1)
  {
    QFont font = goals_list_->item(goal_number+1)->font();
    font.setBold(true);
    goals_list_->item(goal_number+1)->setFont(font);
  }

  ros::Duration time_to_complete = user_demo_.goalComplete();

  QMessageBox box;

  std::stringstream time;
  time << static_cast<boost::posix_time::time_duration>(time_to_complete.toBoost());
  std::string time_str = time.str().substr(0, 8);

  std::stringstream text;
  text << "Goal " << goal_number+1 << " completed in " << time_str << "!";

  if(goal_number+1 >= node_.getSceneManager()->getNumGoals())
  {
    text << " All goals complete!";
    if(user_demo_.started_)
      endBasicMode();
  }
  else
  {
    text << " Next goal:\n" << node_.getSceneManager()->getGoalDescription(goal_number+1);
  }
  
  box.setText(QString(text.str().c_str()));

  box.exec();

  playSimulator();
}

void DemonstrationVisualizer::tabChanged(int index)
{
  if(index == 0)
  {
    advanced_->layout()->removeWidget(goals_list_);
    basic_->layout()->addWidget(goals_list_);
  }
  else if(index == 1)
  {
    basic_->layout()->removeWidget(goals_list_);
    advanced_->layout()->addWidget(goals_list_);
  }
}

void DemonstrationVisualizer::startBasicMode()
{
  start_button_->setEnabled(false);
  end_button_->setEnabled(true);

  // Reset the robot.
  resetRobot();

  // Reset the task.
  resetTask();

  playSimulator();

  setEditGoalsMode(Qt::Unchecked);

  setEditSceneMode(Qt::Unchecked);

  user_demo_.start();
 
  // Select the interaction tool. (@todo make the tools constants somewhere)
  changeTool(2);

  // Begin recording.
  std::string package_path = ros::package::getPath("demonstration_visualizer");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package demonstration_visualizer!");
    return;
  }
  pr2_simple_simulator::FilePath srv;
  srv.request.file_path = package_path;
  if(!node_.beginRecording(srv))
  {
    ROS_ERROR("Failed to call service /motion_recorder/begin_recording.");
    return;
  }
}

void DemonstrationVisualizer::endBasicMode()
{
  end_button_->setEnabled(false);
  start_button_->setEnabled(true);

  pauseSimulator();

  changeTool(1);

  setEditGoalsMode(Qt::Checked);

  setEditSceneMode(Qt::Checked);

  ros::Duration d = user_demo_.stop();

  std::stringstream time;
  time << static_cast<boost::posix_time::time_duration>(d.toBoost());
  std::string time_str = time.str().substr(0, 8);

  // End recording.
  std_srvs::Empty empty;
  node_.endRecording(empty);

  // Show the base path at the end.
  node_.showBasePath();

  ROS_INFO("[DViz] User demonstration ended. Completed in %d goals in %s.",
	   user_demo_.goals_completed_, time_str.c_str());
}

void DemonstrationVisualizer::focusCameraTo(float x, float y, float z)
{
  // Focus the camera to look at the point (x, y, z) relative to the fixed frame.
  rviz::ViewManager *view_manager = visualization_manager_->getViewManager();
  view_manager->getCurrent()->lookAt(x, y, z);
}

void DemonstrationVisualizer::updateCamera(const geometry_msgs::Pose &A, const geometry_msgs::Pose &B)
{
  rviz::ViewManager *view_manager = visualization_manager_->getViewManager();

  if(previous_camera_mode_ == FPS && camera_mode_ != FPS)
  {
    geometry_msgs::Twist vel;
    vel.linear.x = 0;
    vel.linear.y = 0;
    vel.angular.z = 0;

    node_.sendBaseVelocityCommand(vel);
  }

  switch(camera_mode_)
  {
  case ORBIT:
    {
      if(previous_camera_mode_ != ORBIT)
      {
	camera_buttons_[ORBIT]->setEnabled(false);
	camera_buttons_[previous_camera_mode_]->setEnabled(true);

	ROS_INFO("[DViz] Switching to rviz/Orbit view.");
	view_manager->setCurrentViewControllerType("rviz/Orbit");
	view_manager->getCurrent()->subProp("Target Frame")->setValue("base_footprint");
	view_manager->getCurrent()->subProp("Distance")->setValue(4.0);
	view_manager->getCurrent()->subProp("Focal Point")->subProp("X")->setValue(0.0);
	view_manager->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue(0.0);
	view_manager->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue(0.0);
      }
      
      // Ensure that the user never moves the camera below the ground.
      if(view_manager->getCurrent()->subProp("Pitch")->getValue().toDouble() < 0.0)
	view_manager->getCurrent()->subProp("Pitch")->setValue(0.0);

      break;
    }
  case FPS:
    {
      if(previous_camera_mode_ != FPS)
      {
	camera_buttons_[FPS]->setEnabled(false);
	camera_buttons_[previous_camera_mode_]->setEnabled(true);

	ROS_INFO("[DViz] Switching to rviz/FPS view.");
	view_manager->setCurrentViewControllerType("rviz/FPS");
	view_manager->getCurrent()->subProp("Target Frame")->setValue("base_footprint");
	view_manager->getCurrent()->subProp("Position")->subProp("X")->setValue(0.0);
	view_manager->getCurrent()->subProp("Position")->subProp("Y")->setValue(0.0);
	view_manager->getCurrent()->subProp("Position")->subProp("Z")->setValue(1.2);

	view_manager->getCurrent()->subProp("Yaw")->setValue(tf::getYaw(node_.getBasePose().orientation));
	view_manager->getCurrent()->subProp("Pitch")->setValue(0.0);

	// Filter mouse scroll wheel events so that the user cannot zoom in/out.
	view_manager->getCurrent()->installEventFilter(this);
      }

      geometry_msgs::Twist vel_cmd;
      // @todo set a new goal orientation for the base.
      double current_camera_yaw = view_manager->getCurrent()->subProp("Yaw")->getValue().toDouble();
      geometry_msgs::Pose base_pose = node_.getBasePose();
      double current_base_yaw = tf::getYaw(base_pose.orientation);

      // Note that B is always the larger angle. We wish to move in the direction that minimizes
      // the angular distance between the angular position of the camera and the base. These angles
      // are in the range [0, 2\pi), so we need to account for the discontinuity. 
      double A = 0.0;
      double B = 0.0;
      bool base_angle_larger = current_base_yaw > current_camera_yaw;

      if(base_angle_larger)
      {
	B = current_base_yaw;
	A = current_camera_yaw;
      }
      else
      {
	B = current_camera_yaw;
	A = current_base_yaw;
      }

      bool clockwise = (base_angle_larger && (B - A) < (2*M_PI - B + A)) ||
	(!base_angle_larger && (B - A) > (2*M_PI - B + A));

      if((clockwise ? (2*M_PI - B + A) > 0.1 : std::abs(B - A) > 0.1))
      {
	if(clockwise)
	  vel_cmd.angular.z = -0.3;
	else
	  vel_cmd.angular.z = 0.3;
      }
      else
      {
	vel_cmd.angular.z = 0;
      }

      node_.sendBaseVelocityCommand(vel_cmd);

      // Offset the position of the FPS camera, if set.
      if(x_fps_offset_ > 0)
      {
	view_manager->getCurrent()->subProp("Position")->
	  subProp("X")->setValue(base_pose.position.x - x_fps_offset_ * std::cos(current_base_yaw));
	view_manager->getCurrent()->subProp("Position")->
	  subProp("Y")->setValue(base_pose.position.y - x_fps_offset_ * std::sin(current_base_yaw));
      }
      if(z_fps_offset_ > 0)
	view_manager->getCurrent()->subProp("Position")->subProp("Z")->setValue(1.2 + z_fps_offset_);

      if(x_fps_offset_ > 0 && z_fps_offset_ > 0)
	view_manager->getCurrent()->subProp("Pitch")->setValue(std::atan2(z_fps_offset_, x_fps_offset_));

      break;
    }
  case TOP_DOWN:
    {
      if(previous_camera_mode_ != TOP_DOWN)
      {
	camera_buttons_[TOP_DOWN]->setEnabled(false);
	camera_buttons_[previous_camera_mode_]->setEnabled(true);

	ROS_INFO("[DViz] Switching to rviz/Orbit top-down view.");
	view_manager->setCurrentViewControllerType("rviz/Orbit");
	view_manager->getCurrent()->subProp("Target Frame")->setValue("base_footprint");
	view_manager->getCurrent()->subProp("Distance")->setValue(4.0);
	view_manager->getCurrent()->subProp("Focal Point")->subProp("X")->setValue(0.0);
	view_manager->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue(0.0);
	view_manager->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue(0.0);
      }
      view_manager->getCurrent()->subProp("Pitch")->setValue(M_PI/2.0);
      view_manager->getCurrent()->subProp("Yaw")->setValue(tf::getYaw(node_.getBasePose().orientation));

      view_manager->getCurrent()->installEventFilter(this);

      break;
    }
  case AUTO:
    {
      if(previous_camera_mode_ != AUTO)
      {
	camera_buttons_[AUTO]->setEnabled(false);
	camera_buttons_[previous_camera_mode_]->setEnabled(true);

	ROS_INFO("[DViz] Switching to automatic camera control.");
	view_manager->setCurrentViewControllerType("rviz/Orbit");
	view_manager->getCurrent()->subProp("Target Frame")->setValue("map");
	view_manager->getCurrent()->subProp("Distance")->setValue(4.0);
      }

      geometry_msgs::Point midpoint;
      midpoint.x = (A.position.x + B.position.x)/2.0;
      midpoint.y = (A.position.y + B.position.y)/2.0;
      midpoint.z = (A.position.z + B.position.z)/2.0;

      // First focus camera to the appropriate position.
      view_manager->getCurrent()->subProp("Focal Point")->subProp("X")->setValue(midpoint.x);
      view_manager->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue(midpoint.y);
      view_manager->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue(midpoint.z);

      // Next choose the appropriate zoom
      Ogre::Camera *camera = view_manager->getCurrent()->getCamera();
      // Vertical field of view.
      float V = camera->getFOVy().valueRadians();
      //float H = camera->getFOVx().valueRadians();
      // Aspect ratio.
      float r = camera->getAspectRatio();
      // Horizontal field of view.
      float H = 2*std::atan(0.5*std::tan(V) * r);
      float th = std::min(H,V);
      double dx = B.position.x - A.position.x;
      double dy = B.position.y - A.position.y;
      double dz = B.position.z - A.position.z;
      double d = sqrt(dx*dx+dy*dy+dz*dz) + 4.0;
      double zoom = d/(2*tan(th/2.0));
      view_manager->getCurrent()->subProp("Distance")->setValue(zoom);

      //roll is 0 since we always want the camera straight up and down (with respect to gravity)
      // @todo the view controller does not allow us to set the roll.
      //view_manager->getCurrent()->subProp("Roll")->setValue(0.0);

      //choose the pitch and yaw to be on the highest point on the circle orthogonal to the line
      //this is computed by crossing the hand to goal vector with a vector that has no z component and is rotated 90 degress in xy
      double v1x = dx;
      double v1y = dy;
      double v1z = dz;
      double v2x = -v1y;
      double v2y = v1x;
      double v2z = 0;
      double v3x = v1y*v2z - v1z*v2y;
      double v3y = v1z*v2x - v1x*v2z;
      double v3z = v1x*v2y - v1y*v2x;

      double yaw_angle = atan2(v3y,v3x);
      double xy_length = sqrt(v3x*v3x+v3y*v3y);
      double pitch_angle = atan2(v3z, xy_length);

      view_manager->getCurrent()->subProp("Yaw")->setValue(yaw_angle);
      view_manager->getCurrent()->subProp("Pitch")->setValue(pitch_angle);

      //the yaw will be orthogonal to the xy projection of the line connecting the gripper and goal
      //double line_angle = atan2(dy,dx);
      //view_manager->getCurrent()->subProp("Yaw")->setValue(line_angle+M_PI/2.0);
      //view_manager->getCurrent()->subProp("Yaw")->setValue(tf::getYaw(node_.getBasePose().orientation));
      
      //the pitch is going to be a function of the z to xy ratio
      //view_manager->getCurrent()->subProp("Pitch")->setValue(0.0);

      // Then, ... @todo figure out how to position the camera.
      // double dx = B.position.x - A.position.x;
      // double dy = B.position.y - B.position.y;
      // double m = -dx/dy;

      // Ogre::Camera *camera = view_manager->getCurrent()->getCamera();
      // // Vertical field of view.
      // float V = camera->getFOVy().valueRadians();
      // // Aspect ratio.
      // float r = camera->getAspectRatio();
      // // Horizontal field of view.
      // float H = 2*std::atan(std::tan(V) * r);
      // // ROS_INFO_STREAM("Camera vertical FOV = " << (180.0/M_PI) * V << ", horizontal FOV = " 
      // // 		  << (180.0/M_PI) * H << ", aspect ratio = " << r);
      break;
    }
  default:
    break;
  }

    previous_camera_mode_ = camera_mode_;
}

void DemonstrationVisualizer::changeCameraMode(int mode)
{
  ROS_INFO("[DViz] Changing camera mode from %d to %d.", camera_mode_, mode);

  previous_camera_mode_ = (CameraMode)camera_mode_;

  camera_mode_ = (CameraMode)mode;
}

void DemonstrationVisualizer::pauseSimulator()
{
  QPushButton *pause_button = basic_->findChild<QPushButton *>("Pause");
  if(pause_button)
    pause_button->setEnabled(false);
  QPushButton *play_button = basic_->findChild<QPushButton *>("Play");
  if(play_button)
    play_button->setEnabled(true);

  std_srvs::Empty empty;
  if(!node_.pauseSimulator(empty))
    ROS_ERROR("[DViz] Failed to pause simulation!");
}

void DemonstrationVisualizer::playSimulator()
{
  QPushButton *pause_button = basic_->layout()->findChild<QPushButton *>("Pause");
  if(pause_button)
    pause_button->setEnabled(true);
  QPushButton *play_button = basic_->layout()->findChild<QPushButton *>("Play");
  if(play_button)
    play_button->setEnabled(false);

  std_srvs::Empty empty;
  if(!node_.playSimulator(empty))
    ROS_ERROR("[DViz] Failed to play simulation!");
}

void DemonstrationVisualizer::toggleZMode()
{
  if(z_mode_)
    disableZMode();
  else
    enableZMode();
}

void DemonstrationVisualizer::enableZMode()
{
  ROS_INFO("[DViz] Enabling Z-mode.");
  z_mode_ = true;
  z_mode_button_->setText("Disable Z-Mode");
  node_.processKeyEvent(pr2_simple_simulator::KeyEvent::Request::KEY_Z,
			QEvent::KeyPress);
}

void DemonstrationVisualizer::disableZMode()
{
  ROS_INFO("[DViz] Disabling Z-mode.");
  z_mode_ = false;
  z_mode_button_->setText("Enable Z-Mode");
  node_.processKeyEvent(pr2_simple_simulator::KeyEvent::Request::KEY_Z,
			QEvent::KeyRelease);
}

void DemonstrationVisualizer::setFPSXOffset(int offset)
{
  if(camera_mode_ == FPS)
  {
    x_fps_offset_ = (double)offset/100;
    ROS_INFO("[DViz] (FPS mode) Setting x-offset to %f.", x_fps_offset_);
  }
}

void DemonstrationVisualizer::setFPSZOffset(int offset)
{
  if(camera_mode_ == FPS)
  {
    z_fps_offset_ = (double)offset/100;
    ROS_INFO("[DViz] (FPS mode) Setting z-offset to %f.", z_fps_offset_);
  }
}
