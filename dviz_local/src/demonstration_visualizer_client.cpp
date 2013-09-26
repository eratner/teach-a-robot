#include <dviz_local/demonstration_visualizer_client.h>

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
#include <QMenu>

#include <ros/package.h>

#define ICON_SIZE 65

namespace demonstration_visualizer
{

DemonstrationVisualizerClient::DemonstrationVisualizerClient(int argc, char **argv, QWidget *parent)
 : QWidget(parent), user_(argc, argv, 0, false)
{
  setWindowTitle("Teach-A-Robot");

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
  visualization_manager_->setFixedFrame(QString(resolveName("map", 0).c_str()));

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
  scale_mesh_slider_ = new QSlider(Qt::Horizontal);
  scale_mesh_slider_->setMinimum(1);
  scale_mesh_slider_->setMaximum(5000);
  scale_mesh_slider_->setValue(100);
  scale_mesh_panel->addWidget(scale_mesh_slider_);

  rviz::ToolManager *tool_manager = visualization_manager_->getToolManager();
  ROS_INFO("There are %d tools loaded:", tool_manager->numTools());
  for(int i = 0; i < tool_manager->numTools(); ++i)
  {
    ROS_INFO("%d. %s", i, tool_manager->getTool(i)->getClassId().toLocal8Bit().data());
    select_tool->addItem(tool_manager->getTool(i)->getClassId());
  }

  // Set camera (view) properties.
  rviz::ViewManager *view_manager = visualization_manager_->getViewManager();
  ROS_INFO("Target Frame = %s", resolveName("base_footprint", 0).c_str());
  view_manager->getCurrent()->subProp("Target Frame")->setValue(QVariant(QString(resolveName("base_footprint", 0).c_str())));

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
  linear_speed_control->setValue(0.4);

  QHBoxLayout *angular_speed_panel = new QHBoxLayout();
  QLabel *angular_speed_label = new QLabel("Angular Speed (rad/s): ");
  angular_speed_panel->addWidget(angular_speed_label);
  QDoubleSpinBox *angular_speed_control = new QDoubleSpinBox();
  angular_speed_panel->addWidget(angular_speed_control);
  angular_speed_control->setSingleStep(0.1);
  angular_speed_control->setValue(0.4);

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
  goals_list_->setContextMenuPolicy(Qt::CustomContextMenu);
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

  QGroupBox *controls_group = new QGroupBox();
  QVBoxLayout *user_controls_layout = new QVBoxLayout();

  QHBoxLayout *gripper_controls_layout = new QHBoxLayout();

  start_stop_button_ = new QPushButton();
  QPixmap start_stop_pixmap(":/icons/start.png");
  QIcon start_stop_icon(start_stop_pixmap);
  start_stop_button_->setIcon(start_stop_icon);
  start_stop_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
  gripper_controls_layout->addWidget(start_stop_button_);

  play_pause_button_ = new QPushButton();
  QPixmap play_pause_pixmap(":/icons/play.png");
  QIcon play_pause_icon(play_pause_pixmap);
  play_pause_button_->setIcon(play_pause_icon);
  play_pause_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
  gripper_controls_layout->addWidget(play_pause_button_);  

  accept_change_grasp_button_ = new QPushButton();
  QPixmap accept_change_grasp_pixmap(":/icons/accept_grasp.png");
  QIcon accept_change_grasp_icon(accept_change_grasp_pixmap);
  accept_change_grasp_button_->setIcon(accept_change_grasp_icon);
  accept_change_grasp_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
  accept_change_grasp_button_->setEnabled(false);
  gripper_controls_layout->addWidget(accept_change_grasp_button_);

  gripper_orientation_button_ = new QPushButton();
  QPixmap gripper_orientation_pixmap(":/icons/full_control.png");
  QIcon gripper_orientation_icon(gripper_orientation_pixmap);
  gripper_orientation_button_->setIcon(gripper_orientation_icon);
  gripper_orientation_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
  gripper_controls_layout->addWidget(gripper_orientation_button_);

  toggle_collisions_button_ = new QPushButton();
  QPixmap toggle_collisions_pixmap(":/icons/disable2_collisions.png");
  QIcon toggle_collisions_icon(toggle_collisions_pixmap);
  toggle_collisions_button_->setIcon(toggle_collisions_icon);
  toggle_collisions_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
  gripper_controls_layout->addWidget(toggle_collisions_button_);

  user_controls_layout->addLayout(gripper_controls_layout);

  QHBoxLayout *grasp_distance_layout = new QHBoxLayout();
  QLabel *grasp_distance_label = new QLabel("Grasp Distance: ");
  grasp_distance_slider_ = new QSlider(Qt::Horizontal);
  grasp_distance_layout->addWidget(grasp_distance_label);
  grasp_distance_slider_->setMinimum(0);
  grasp_distance_slider_->setMaximum(100);
  grasp_distance_slider_->setValue(25);
  grasp_distance_slider_->setEnabled(false);
  grasp_distance_layout->addWidget(grasp_distance_slider_);
  user_controls_layout->addLayout(grasp_distance_layout);

  QHBoxLayout *gripper_position_layout = new QHBoxLayout();
  QLabel *gripper_position_label = new QLabel("Grasp Position: ");
  gripper_position_layout->addWidget(gripper_position_label);
  gripper_position_slider_ = new QSlider(Qt::Horizontal);
  // @todo make these constants.
  gripper_position_slider_->setMinimum(0);
  gripper_position_slider_->setMaximum(548);
  gripper_position_slider_->setValue(0);
  gripper_position_slider_->setEnabled(false);
  gripper_position_layout->addWidget(gripper_position_slider_);
  user_controls_layout->addLayout(gripper_position_layout);

  QGroupBox *camera_group = new QGroupBox();
  QHBoxLayout *camera_controls = new QHBoxLayout();
  QPushButton *orbit_camera = new QPushButton("Orbit\nCamera");
  camera_buttons_.push_back(orbit_camera);
  QPushButton *fps_camera = new QPushButton("FPS\nCamera");
  camera_buttons_.push_back(fps_camera);
  QPushButton *top_down_camera = new QPushButton ("Top Down\nCamera");
  camera_buttons_.push_back(top_down_camera);
  QPushButton *auto_camera = new QPushButton("Auto\nCamera");
  camera_buttons_.push_back(auto_camera);
  QPushButton *top_down_fps_camera = new QPushButton("Top Down/FPS\nCamera");
  camera_buttons_.push_back(top_down_fps_camera);

  camera_controls->addWidget(orbit_camera);
  camera_controls->addWidget(fps_camera);
  camera_controls->addWidget(auto_camera);
  camera_controls->addWidget(top_down_fps_camera);
  camera_group->setLayout(camera_controls);

  controls_group->setLayout(user_controls_layout);

  basic_layout->addWidget(controls_group);

  basic_layout->addWidget(camera_group);

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
  grid_->setEnabled(false);

  std::stringstream ss;

  // Create a robot model display.
  robot_model_ = visualization_manager_->createDisplay("rviz/MarkerArray", "Robot", true);
  ROS_ASSERT(robot_model_ != NULL);
  ROS_INFO("Marker Topic = %s", resolveName("visualization_marker_array", 0).c_str());
  robot_model_->subProp("Marker Topic")->setValue(QVariant(QString(resolveName("visualization_marker_array", 0).c_str())));

  // Create an interactive markers display.
  robot_interactive_markers_ = visualization_manager_->createDisplay("rviz/InteractiveMarkers", 
								     "Interactive Markers", 
								     true);
  ROS_ASSERT(robot_interactive_markers_ != NULL);
  ss << resolveName("interactive_markers", 0) << "/update";
  robot_interactive_markers_->subProp("Update Topic")->setValue(QVariant(QString(ss.str().c_str())));

  // Create a visualization markers display for scenes of demonstration environments.
  visualization_marker_ = visualization_manager_->createDisplay("rviz/Marker", "Scene", true);
  visualization_marker_->subProp("Marker Topic")->setValue(QVariant(QString(resolveName("visualization_marker", 0).c_str())));
  ROS_ASSERT(visualization_marker_ != NULL);

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
  //connect(&core_, SIGNAL(goalComplete(int)), this, SLOT(notifyGoalComplete(int)));
  connect(goals_list_, 
	  SIGNAL(itemDoubleClicked(QListWidgetItem *)), this, 
	  SLOT(editGoalDescription(QListWidgetItem *))
	  );
  connect(goals_list_,
	  SIGNAL(customContextMenuRequested(const QPoint &)), this,
	  SLOT(showGoalsMenu(const QPoint &)));
  connect(tabs, SIGNAL(currentChanged(int)), this, SLOT(tabChanged(int)));
  connect(scale_mesh_slider_, SIGNAL(valueChanged(int)), this, SLOT(scaleMesh(int)));

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
  connect(top_down_fps_camera, SIGNAL(clicked()), camera_signal_mapper, SLOT(map()));
  camera_signal_mapper->setMapping(top_down_fps_camera, (int)TOP_DOWN_FPS);

  // connect(&core_, SIGNAL(updateCamera(const geometry_msgs::Pose &, const geometry_msgs::Pose &)), this, 
  // 	  SLOT(updateCamera(const geometry_msgs::Pose &, const geometry_msgs::Pose &)));

  connect(start_stop_button_, SIGNAL(clicked()), this, SLOT(toggleStartStop()));
  connect(play_pause_button_, SIGNAL(clicked()), this, SLOT(togglePlayPause()));
  connect(accept_change_grasp_button_, SIGNAL(clicked()), this, SLOT(acceptChangeGrasp()));
  connect(gripper_orientation_button_, SIGNAL(clicked()), this, SLOT(toggleGripperOrientationMode()));
  connect(grasp_distance_slider_, SIGNAL(valueChanged(int)), this, SLOT(setGraspDistance(int)));
  connect(gripper_position_slider_, SIGNAL(valueChanged(int)), this, SLOT(setGripperPosition(int)));
  connect(toggle_collisions_button_, SIGNAL(clicked()), this, SLOT(toggleCollisions()));

  next_mesh_id_ = 3;
  selected_mesh_ = -1;
  previous_camera_mode_ = camera_mode_ = ORBIT;
  z_mode_ = false;
  x_fps_offset_ = 0;
  z_fps_offset_ = 0;
  current_state_ = NORMAL;
  top_down_fps_camera_mode_ = 0;
  last_top_down_fps_camera_mode_ = 1;
  grasp_selected_ = false;

  gripper_orientation_mode_ = false;
  collisions_mode_ = true;
  started_ = false;
  playing_ = true;
  accepted_grasp_ = false;

  setLayout(window_layout);

  // Start the thread to run the DVizUser associated with this DVizClient.
  dviz_user_thread_ = boost::thread(&DemonstrationVisualizerUser::run, &user_);

  resetRobot();
}

DemonstrationVisualizerClient::~DemonstrationVisualizerClient()
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

void DemonstrationVisualizerClient::changeState(State s)
{
  // @todo print state transition.

  current_state_ = s;
}

DemonstrationVisualizerClient::State DemonstrationVisualizerClient::getState() const
{
  return current_state_;
}

void DemonstrationVisualizerClient::keyPressEvent(QKeyEvent *event)
{
  if(event->isAutoRepeat())
    event->ignore();
  else
  {
    // Use the spacebar to toggle between top down and FPS in the 
    // Top Down/FPS mode.
    if(camera_mode_ == TOP_DOWN_FPS &&
       event->key() == Qt::Key_Space)
    {
      if(top_down_fps_camera_mode_ == 0)
      {
	last_top_down_fps_camera_mode_ = 0;
	top_down_fps_camera_mode_ = 1;
      }
      else
      {
	last_top_down_fps_camera_mode_ = 1;
	top_down_fps_camera_mode_ = 0;
      }
    }
    else
    {
      std::vector<std::string> args;
      args.push_back(boost::lexical_cast<std::string>(event->key()));
      args.push_back(boost::lexical_cast<std::string>(QEvent::KeyPress));
      if(!user_.processCommand("process_key", args))
      {
	ROS_ERROR("[DVizClient] Failed to process key event!");
      }
      QWidget::keyPressEvent(event);
    }
  }
}

void DemonstrationVisualizerClient::keyReleaseEvent(QKeyEvent *event)
{
  if(event->isAutoRepeat())
    event->ignore();
  else
  {
    std::vector<std::string> args;
    args.push_back(boost::lexical_cast<std::string>(event->key()));
    args.push_back(boost::lexical_cast<std::string>(QEvent::KeyRelease));
    if(!user_.processCommand("process_key", args))
    {
      ROS_ERROR("[DVizClient] Failed to process key event!");
    }
    QWidget::keyReleaseEvent(event);
  }
}

bool DemonstrationVisualizerClient::eventFilter(QObject *obj, QEvent *event)
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
    if(camera_mode_ == FPS || camera_mode_ == TOP_DOWN || camera_mode_ == TOP_DOWN_FPS)
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
  else if(event->type() == QEvent::MouseMove)
  {
    if(camera_mode_ == TOP_DOWN)
    {
      return true;
    }
    else
      return QObject::eventFilter(obj, event);
  }
  else if(event->type() == QEvent::MouseButtonPress)
  {
    QMouseEvent *mouse_event = static_cast<QMouseEvent *>(event);
    switch(mouse_event->button())
    {
    case Qt::MiddleButton:
    {
      if(camera_mode_ != ORBIT)
      {
	return true;
      }

      break;
    }
    case Qt::RightButton:
    {
      if(camera_mode_ != ORBIT)
      {
	return true;
      }

      break;
    }
    default:
      break;
    }

    return QObject::eventFilter(obj, event);
  }
  else if(event->type() == QEvent::MouseButtonRelease)
  {
    // QMouseEvent *mouse_event = static_cast<QMouseEvent *>(event);
    // switch(mouse_event->button())
    // {
    // case Qt::MiddleButton:
    //   {
    // 	if(camera_mode_ == FPS ||
    // 	   camera_mode_ == TOP_DOWN ||
    // 	   camera_mode_ == AUTO ||
    // 	   camera_mode_ == GOAL)
    // 	{
    // 	  return true;
    // 	}

    // 	break;
    //   }
    // default:
    //   break;
    // }

    return QObject::eventFilter(obj, event);
  }
  else
  {
    return QObject::eventFilter(obj, event);
  }
}

void DemonstrationVisualizerClient::toggleGrid()
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

void DemonstrationVisualizerClient::resetRobot()
{
  // user_.resetRobot();
  if(!user_.processCommand("reset_robot", std::vector<std::string>()))
  {
    ROS_ERROR("[DVizClient] Failed to reset robot!");
  }
}

void DemonstrationVisualizerClient::resetTask()
{
  user_demo_.goals_completed_ = 0;

  // user_.getSceneManager()->setCurrentGoal(0);
  // user_.getSceneManager()->resetTask();
  if(!user_.processCommand("reset_task", std::vector<std::string>()))
  {
    ROS_ERROR("[DVizClient] Failed to reset task!");
    return;
  }

  // Reset the goal list.
  for(int i = 0; i < user_.getSceneManager()->getNumGoals(); ++i)
  {
    QFont font = goals_list_->item(i)->font();
    font.setBold(i == 0 ? true : false);
    font.setStrikeOut(false);
    goals_list_->item(i)->setFont(font);     
  }

  // user_.getSceneManager()->setGoalsChanged();
}

void DemonstrationVisualizerClient::changeTool(int tool_index)
{
  if(tool_index == 0)
    return;

  rviz::ToolManager *tool_manager = visualization_manager_->getToolManager();

  tool_manager->setCurrentTool(tool_manager->getTool(tool_index-1));

  ROS_INFO("Current tool changed to: %s",
	   visualization_manager_->getToolManager()->getCurrentTool()->getName().toLocal8Bit().data());  
}

void DemonstrationVisualizerClient::beginRecording()
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

  std::vector<std::string> args;
  args.push_back(directory.toStdString());
  // user_.getMotionRecorder()->beginRecording(directory.toStdString());
  if(!user_.processCommand("begin_recording", args))
  {
    ROS_ERROR("[DVizClient] Failed to begin recording from %s", args[0].c_str());
    return;
  }

  recording_icon_->show();

  ROS_INFO("[DVizClient] Recording to %s", directory.toLocal8Bit().data());
}

void DemonstrationVisualizerClient::endRecording()
{
  // user_.getMotionRecorder()->endRecording();

  recording_icon_->hide();

  ROS_INFO("[DVizClient] Recording has ended.");
}

void DemonstrationVisualizerClient::beginReplay()
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

  // user_.getMotionRecorder()->beginReplay(filename.toStdString());
  std::vector<std::string> args;
  args.push_back(filename.toStdString());
  if(!user_.processCommand("begin_replay", args))
  {
    ROS_ERROR("[DVizClient] Failed to begin replay from %s.", args[0].c_str());
    return;
  }

  // user_.playSimulator();
  playSimulator();

  replaying_icon_->show();

  ROS_INFO("Replaying from file %s", filename.toLocal8Bit().data());
}

void DemonstrationVisualizerClient::endReplay()
{
  // user_.getMotionRecorder()->endReplay();

  replaying_icon_->hide();
}

void DemonstrationVisualizerClient::loadMesh()
{
  std::string package_path = ros::package::getPath("dviz_core");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package dviz_core!");
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

  std::size_t found = filename.toStdString().find("dviz_core");
  if(found == std::string::npos)
  {
    ROS_ERROR("[DViz] Please load a mesh that is within a subdirectory of the dviz_core pacakge.");
    return;
  }

  bool movable = false;
  if(QMessageBox::Yes == QMessageBox::question(this, "Movable?", 
					       "Is this object movable?", 
					       QMessageBox::Yes | QMessageBox::No))
  {
    movable = true;
  }

  ROS_INFO_STREAM("[DViz] Movable? " << (movable ? "Yes." : "No."));

  std::stringstream resource_path;
  resource_path << "package://" << filename.toStdString().substr(found);
  ROS_INFO("Loading mesh from file %s.", resource_path.str().c_str());

  int i = resource_path.str().size()-1;
  for(; i >= 0; i--)
  {
    if(resource_path.str()[i] == '/')
      break;
  }

  int offset = 0;
  for(int j = i; j < int(resource_path.str().size()); ++j)
  {
    if(resource_path.str()[j] == '.')
      break;
    else
      offset++;
  }

  std::string mesh_name = resource_path.str().substr(i+1, offset-1);
  ROS_INFO("[DViz] Assigning the name \"%s\".", mesh_name.c_str());
  mesh_names_.insert(std::pair<int, std::string>(next_mesh_id_, resource_path.str()));
  next_mesh_id_++;

  select_mesh_->addItem(QString(mesh_name.c_str()), QVariant(next_mesh_id_-1));

  // user_.getSceneManager()->addMeshFromFile(resource_path.str(), next_mesh_id_-1, mesh_name, movable);
  std::vector<std::string> args;
  args.push_back(resource_path.str());
  args.push_back(mesh_name);
  args.push_back((movable ? "true" : "false"));
  // @todo ID!!
  user_.processCommand("load_mesh", args);
}

void DemonstrationVisualizerClient::deleteMesh()
{
  if(selected_mesh_ == -1)
  {
    ROS_INFO("[DViz] No marker selected.");
    return;
  }

  ROS_INFO("Deleting mesh %d.", selected_mesh_);

  user_.getSceneManager()->removeMesh(selected_mesh_);

  // @todo it would be cleaner to let the demonstration scene manager handle this.
  select_mesh_->removeItem(select_mesh_->findData(QVariant(selected_mesh_)));
  std::map<int, std::string>::iterator it = mesh_names_.find(selected_mesh_);
  if(it != mesh_names_.end())
    mesh_names_.erase(mesh_names_.find(selected_mesh_));

  if(mesh_names_.size() == 0)
    selected_mesh_ = -1;
}

void DemonstrationVisualizerClient::setEditSceneMode(int mode)
{
  switch(mode)
  {
  case Qt::Unchecked:
  {
    user_.getSceneManager()->setEditMeshesMode(false);

    break;
  }
  case Qt::Checked:
  {
    user_.getSceneManager()->setEditMeshesMode(true);

    break;
  }
  default:
    ROS_ERROR("[DViz] Invalid edit scene mode!");
    break;
  }
}

void DemonstrationVisualizerClient::loadScene()
{
  std::string package_path = ros::package::getPath("dviz_core");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package dviz_core!");
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
    pauseSimulator();

    // Load the scene into the demonstration scene manager.
    int max_mesh_id = user_.getSceneManager()->loadScene(filename.toStdString());

    if(max_mesh_id < 0)
    {
      ROS_ERROR("[DViz] Failed to load scene from %s!", filename.toStdString().c_str());
      return;
    }

    // Visualize each mesh marker.
    std::vector<visualization_msgs::Marker> meshes = user_.getSceneManager()->getMeshes();
    std::vector<visualization_msgs::Marker>::iterator it;
    for(it = meshes.begin(); it != meshes.end(); ++it)
    {
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

void DemonstrationVisualizerClient::saveScene()
{
  std::string package_path = ros::package::getPath("dviz_core");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package dviz_core!");
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

  user_.getSceneManager()->saveScene(filename.toStdString());
}

void DemonstrationVisualizerClient::loadTask()
{
  std::string package_path = ros::package::getPath("dviz_core");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package dviz_core!");
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
    user_.getSceneManager()->loadTask(filename.toStdString());
    goals_list_->clear();
    std::vector<Goal *> goals = user_.getSceneManager()->getGoals();
    for(int i = 0; i < (int)goals.size(); ++i)
    {
      std::stringstream goal_desc;
      goal_desc << "Goal " << i+1 << ": " << goals[i]->getDescription();
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

void DemonstrationVisualizerClient::saveTask()
{
  std::string package_path = ros::package::getPath("dviz_core");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package dviz_core!");
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

  user_.getSceneManager()->saveTask(filename.toStdString());
}

void DemonstrationVisualizerClient::setEditGoalsMode(int mode)
{
  switch(mode)
  {
  case Qt::Unchecked:
    user_.getSceneManager()->setEditGoalsMode(false);
    user_.getSceneManager()->setGoalsChanged();
    break;
  case Qt::Checked:
    user_.getSceneManager()->setEditGoalsMode(true);
    user_.getSceneManager()->setGoalsChanged();
    break;
  default:
    ROS_ERROR("[DViz] Invalid edit goals mode!");
    break;
  }
}

void DemonstrationVisualizerClient::selectMesh(int mesh_index)
{
  mesh_index -= 1;
  if(mesh_index >= 0)
  {
    ROS_INFO("[DViz] Selected mesh %d.", (int)select_mesh_->itemData(mesh_index+1).value<int>());
    selected_mesh_ = select_mesh_->itemData(mesh_index+1).value<int>();

    scale_mesh_slider_->setValue((int)(user_.getSceneManager()->getMeshMarker(selected_mesh_).scale.x * 1000.0));
  }
}

void DemonstrationVisualizerClient::scaleMesh(int value)
{
  if(selected_mesh_ == -1)
  {
    ROS_INFO("[DViz] No marker selected to scale.");
    return;
  }

  double scale_factor = value/1000.0;

  ROS_INFO("[DViz] Scaling mesh %d by a factor of %f.", 
	   select_mesh_->itemData(select_mesh_->currentIndex()).value<int>(),
	   scale_factor);

  user_.getSceneManager()->updateMeshScale(selected_mesh_, scale_factor, scale_factor, scale_factor);
}

void DemonstrationVisualizerClient::setLinearSpeed(double linear)
{
  // user_.setRobotSpeed(linear, 0);
}

void DemonstrationVisualizerClient::setAngularSpeed(double angular)
{
  // user_.setRobotSpeed(0, angular);
}

void DemonstrationVisualizerClient::addTaskGoal()
{
  // Update the object list.
  add_goal_dialog_.setObjects(user_.getSceneManager()->getObjects());

  int value = add_goal_dialog_.exec();
  if(value == QDialog::Rejected)
  {
    ROS_INFO("[DViz] No goal added.");
    return;
  }

  std::string description = add_goal_dialog_.getDescription();
  Goal::GoalType type = add_goal_dialog_.getGoalType();
  int object_id = add_goal_dialog_.getObject();
  bool ignore_yaw = add_goal_dialog_.ignoreYaw();

  ROS_INFO("[DViz] Adding goal of type %s with description \"%s\" and object id %d.",
	   Goal::GoalTypeNames[type], description.c_str(), object_id);
  if(type == Goal::PLACE)
  {
    ROS_INFO_STREAM("[DViz] Ignore yaw? " << (ignore_yaw ? "Yes. " : "No. "));
  }


  if(user_.getSceneManager()->getNumGoals() == 0)
    goals_list_->clear();

  user_.getSceneManager()->addGoal(description, type, object_id, ignore_yaw);

  std::stringstream goal_desc;
  goal_desc << "Goal " << user_.getSceneManager()->getNumGoals() << ": " << description;
  goals_list_->addItem(QString(goal_desc.str().c_str()));
}

void DemonstrationVisualizerClient::editGoalDescription(QListWidgetItem *goal)
{
  int goal_number = goals_list_->row(goal);
  std::string current_desc = user_.getSceneManager()->getGoalDescription(goal_number);

  bool ok;
  QString new_desc = QInputDialog::getText(this,
					   tr("Edit Goal Description:"),
					   tr("Description:"),
					   QLineEdit::Normal,
					   QString(current_desc.c_str()),
					   &ok);
  if(ok)
  {
    user_.getSceneManager()->setGoalDescription(goal_number, new_desc.toStdString());
    std::stringstream goal_desc;
    goal_desc << "Goal " << goal_number+1 << ": " << new_desc.toStdString();
    goal->setData(Qt::DisplayRole, QString(goal_desc.str().c_str()));
  }
}

void DemonstrationVisualizerClient::showGoalsMenu(const QPoint &p)
{
  QPoint global_pos = goals_list_->mapToGlobal(p);
  QModelIndex i = goals_list_->indexAt(p);
  goals_list_->item(i.row())->setSelected(true);

  QMenu menu;
  menu.addAction("Make current goal");

  QAction *selected = menu.exec(global_pos);
  if(selected)
  {
    // @todo in the future if there are more than one menu items, we need to
    // check here which action was selected.
    ROS_INFO("[DViz] Changing goal number %d to the current goal.", (int)i.row());
    int current_goal = user_.getSceneManager()->getCurrentGoal();
    int next_goal = static_cast<int>(i.row());

    // Update the task list.
    QFont font = goals_list_->item(current_goal)->font();
    font.setBold(false);
    goals_list_->item(current_goal)->setFont(font);
    font = goals_list_->item(next_goal)->font();
    font.setBold(true);
    font.setStrikeOut(false);
    goals_list_->item(next_goal)->setFont(font);
    
    // Set the current goal appropriately.
    user_.getSceneManager()->setCurrentGoal(next_goal);
    switch(user_.getSceneManager()->getGoal(next_goal)->getType())
    {
    case Goal::PICK_UP:
    {
      ROS_INFO("[DViz] Switching to pick up goal.");

      QPixmap accept_change_grasp_pixmap(":/icons/accept_grasp.png");
      QIcon accept_change_grasp_icon(accept_change_grasp_pixmap);
      accept_change_grasp_button_->setIcon(accept_change_grasp_icon);
      accept_change_grasp_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
      accept_change_grasp_button_->setEnabled(true);
      accepted_grasp_ = false;

      beginGraspSelection();
	
      break;
    }
    case Goal::PLACE:
    {
      ROS_INFO("[DViz] Switching to place goal.");

      accept_change_grasp_button_->setEnabled(false);

      break;
    }
    default:
      break;
    }
  }
  else
  {

  }
}

void DemonstrationVisualizerClient::notifyGoalComplete(int goal_number)
{
  // user_.pauseSimulatorLater();
  pauseSimulator(true);

  bool done = false;

  // Update the task list.
  QFont font = goals_list_->item(goal_number)->font();
  font.setBold(false);
  font.setStrikeOut(true);
  goals_list_->item(goal_number)->setFont(font);   

  if(goal_number < user_.getSceneManager()->getNumGoals()-1)
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

  if(goal_number+1 >= user_.getSceneManager()->getNumGoals())
  {
    text << " All goals complete!";
    done = true;
  }
  else
  {
    text << " Next goal:\n" << user_.getSceneManager()->getGoalDescription(goal_number+1);
  }

  box.setText(QString(text.str().c_str()));

  box.exec();

  // If all goals are completed, then end the game.
  if(done && user_demo_.started_)
  {
    endBasicMode();
    return;
  }

  playSimulator();

  // The next state of the game is determined by the type of the next goal.
  switch(user_.getSceneManager()->getGoal(goal_number + 1)->getType())
  {
  case Goal::PICK_UP:
  {
    ROS_INFO("Next goal: pick up.");

    QPixmap accept_change_grasp_pixmap(":/icons/accept_grasp.png");
    QIcon accept_change_grasp_icon(accept_change_grasp_pixmap);
    accept_change_grasp_button_->setIcon(accept_change_grasp_icon);
    accept_change_grasp_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
    accept_change_grasp_button_->setEnabled(true);
    accepted_grasp_ = false;

    beginGraspSelection();
	
    break;
  }
  case Goal::PLACE:
  {
    ROS_INFO("Next goal: place.");

    accept_change_grasp_button_->setEnabled(false);

    break;
  }
  default:
    break;
  }
}

void DemonstrationVisualizerClient::tabChanged(int index)
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

void DemonstrationVisualizerClient::startBasicMode()
{
  if(user_.getSceneManager()->getNumGoals() == 0)
  {
    QMessageBox no_task_box;

    no_task_box.setText("There are no goals in the current task!");

    no_task_box.exec();

    return;
  }

  QPixmap start_stop_pixmap(":/icons/done.png");
  QIcon start_stop_icon(start_stop_pixmap);
  start_stop_button_->setIcon(start_stop_icon);
  start_stop_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));

  // Reset the robot.
  resetRobot();

  // Reset the task.
  resetTask();

  // Give the user basic instructions.
  QMessageBox instructions_box;

  instructions_box.setText("<b>Instructions:</b> For each goal in the given task:"
			   "<ul>"
			   "<li>Place the gripper so that it may grasp the object.</li>"
			   "<li>Press <b>Accept Grasp</b>.</li>"
			   "<li>Move the base and gripper to that grasp.</li>"
			   "</ul>");

  instructions_box.exec();

  playSimulator();

  beginGraspSelection();

  setEditGoalsMode(Qt::Unchecked);

  setEditSceneMode(Qt::Unchecked);

  user_demo_.start();

  // Select the interaction tool. (@todo make the tools constants somewhere)
  changeTool(2);

  // Begin recording.
  std::string package_path = ros::package::getPath("dviz_core");

  if(package_path.empty())
  {
    ROS_ERROR("[DViz] Failed to find path to package dviz_core!");
    return;
  }

  // user_.getMotionRecorder()->beginRecording(package_path);
  std::vector<std::string> args;
  args.push_back(package_path);
  if(!user_.processCommand("begin_recording", args))
  {
    ROS_ERROR("[DVizClient] Failed to begin recording to %s!", package_path.c_str());
  }
}

 void DemonstrationVisualizerClient::endBasicMode()
 {
   QPixmap start_stop_pixmap(":/icons/start.png");
   QIcon start_stop_icon(start_stop_pixmap);
   start_stop_button_->setIcon(start_stop_icon);
   start_stop_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));

   pauseSimulator(true);

   changeTool(1);

   setEditGoalsMode(Qt::Checked);

   setEditSceneMode(Qt::Checked);

   ros::Duration d = user_demo_.stop();

   std::stringstream time;
   time << static_cast<boost::posix_time::time_duration>(d.toBoost());
   std::string time_str = time.str().substr(0, 8);

   // End recording.
   if(!user_.processCommand("end_recording", std::vector<std::string>()))
   {
     ROS_ERROR("[DVizClient] Failed to end recording!");
   }

   // Show the base path at the end.
   user_.showBasePath();

   ROS_INFO("[DVizClient] User demonstration ended. Completed in %d goals in %s.",
	    user_demo_.goals_completed_, time_str.c_str());
 }

void DemonstrationVisualizerClient::updateCamera(const geometry_msgs::Pose &A, const geometry_msgs::Pose &B)
{
  // rviz::ViewManager *view_manager = visualization_manager_->getViewManager();

  // if(previous_camera_mode_ == FPS && camera_mode_ != FPS)
  // {
  //   geometry_msgs::Twist vel;
  //   vel.linear.x = 0;
  //   vel.linear.y = 0;
  //   vel.angular.z = 0;

  //   // user_.setBaseVelocity(vel);
  // }

  // switch(camera_mode_)
  // {
  // case ORBIT:
  // {
  //   if(previous_camera_mode_ != ORBIT)
  //   {
  //     camera_buttons_[ORBIT]->setEnabled(false);
  //     if(previous_camera_mode_ != GOAL)
  // 	camera_buttons_[previous_camera_mode_]->setEnabled(true);

  //     ROS_INFO("[DViz] Switching to rviz/Orbit view.");
  //     view_manager->setCurrentViewControllerType("rviz/Orbit");
  //     view_manager->getCurrent()->subProp("Target Frame")->setValue("base_footprint");
  //     view_manager->getCurrent()->subProp("Distance")->setValue(4.0);
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("X")->setValue(0.0);
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue(0.0);
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue(0.0);
  //   }

  //   // Ensure that the user never moves the camera below the ground.
  //   if(view_manager->getCurrent()->subProp("Pitch")->getValue().toDouble() < 0.0)
  //     view_manager->getCurrent()->subProp("Pitch")->setValue(0.0);

  //   break;
  // }
  // case FPS:
  // {
  //   if(previous_camera_mode_ != FPS)
  //   {
  //     camera_buttons_[FPS]->setEnabled(false);
  //     if(previous_camera_mode_ != GOAL)
  // 	camera_buttons_[previous_camera_mode_]->setEnabled(true);

  //     ROS_INFO("[DViz] Switching to rviz/FPS view.");
  //     view_manager->setCurrentViewControllerType("rviz/FPS");
  //     view_manager->getCurrent()->subProp("Target Frame")->setValue("base_footprint");
  //     view_manager->getCurrent()->subProp("Position")->subProp("X")->setValue(0.0);
  //     view_manager->getCurrent()->subProp("Position")->subProp("Y")->setValue(0.0);
  //     view_manager->getCurrent()->subProp("Position")->subProp("Z")->setValue(1.5);

  //     // view_manager->getCurrent()->subProp("Yaw")->setValue(tf::getYaw(user_.getBasePose().orientation))
  // 	;
  //     view_manager->getCurrent()->subProp("Pitch")->setValue(0.0);

  //     // Filter mouse scroll wheel events so that the user cannot zoom in/out.
  //     view_manager->getCurrent()->installEventFilter(this);
  //   }

  //   geometry_msgs::Twist vel_cmd;
  //   // @todo set a new goal orientation for the base.
  //   double current_camera_yaw = view_manager->getCurrent()->subProp("Yaw")->getValue().toDouble();
  //   // geometry_msgs::Pose base_pose = user_.getBasePose();
  //   double current_base_yaw = tf::getYaw(base_pose.orientation) < 0 ? 
  //     tf::getYaw(base_pose.orientation) + 2*M_PI : tf::getYaw(base_pose.orientation);

  //   // Note that B is always the larger angle. We wish to move in the direction that minimizes
  //   // the angular distance between the angular position of the camera and the base. These angles
  //   // are in the range [0, 2\pi), so we need to account for the discontinuity. 
  //   double A = 0.0;
  //   double B = 0.0;
  //   bool base_angle_larger = current_base_yaw > current_camera_yaw;

  //   if(base_angle_larger)
  //   {
  //     B = current_base_yaw;
  //     A = current_camera_yaw;
  //   }
  //   else
  //   {
  //     B = current_camera_yaw;
  //     A = current_base_yaw;
  //   }

  //   bool clockwise = (base_angle_larger && (B - A) < (2*M_PI - B + A)) ||
  //     (!base_angle_larger && (B - A) > (2*M_PI - B + A));

  //   if((clockwise ? (2*M_PI - B + A) > 0.1 : std::abs(B - A) > 0.1))
  //   {
  //     if(clockwise)
  // 	vel_cmd.angular.z = -0.3;
  //     else
  // 	vel_cmd.angular.z = 0.3;
  //   }
  //   else
  //   {
  //     vel_cmd.angular.z = 0;
  //   }

  //   // user_.setBaseVelocity(vel_cmd);

  //   // Offset the position of the FPS camera, if set.
  //   if(x_fps_offset_ > 0)
  //   {
  //     view_manager->getCurrent()->subProp("Position")->
  // 	subProp("X")->setValue(-1.0 * x_fps_offset_);
  //   }
  //   if(z_fps_offset_ > 0)
  //     view_manager->getCurrent()->subProp("Position")->subProp("Z")->setValue(1.5 + z_fps_offset_);

  //   if(x_fps_offset_ > 0 && z_fps_offset_ > 0)
  //     view_manager->getCurrent()->subProp("Pitch")->setValue(std::atan2(z_fps_offset_, x_fps_offset_));

  //   break;
  // }
  // case TOP_DOWN:
  // {
  //   if(previous_camera_mode_ != TOP_DOWN)
  //   {
  //     camera_buttons_[TOP_DOWN]->setEnabled(false);
  //     if(previous_camera_mode_ != GOAL)
  // 	camera_buttons_[previous_camera_mode_]->setEnabled(true);

  //     ROS_INFO("[DViz] Switching to rviz/Orbit top-down view.");
  //     view_manager->setCurrentViewControllerType("rviz/Orbit");
  //     view_manager->getCurrent()->subProp("Target Frame")->setValue("base_footprint");
  //     view_manager->getCurrent()->subProp("Distance")->setValue(4.0);
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("X")->setValue(0.0);
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue(0.0);
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue(0.0);

  //     view_manager->getCurrent()->installEventFilter(this);
  //   }

  //   view_manager->getCurrent()->subProp("Pitch")->setValue(M_PI/2.0);
  //   // view_manager->getCurrent()->subProp("Yaw")->setValue(tf::getYaw(user_.getBasePose().orientation));

  //   break;
  // }
  // case AUTO:
  // {
  //   if(previous_camera_mode_ != AUTO)
  //   {
  //     camera_buttons_[AUTO]->setEnabled(false);
  //     if(previous_camera_mode_ != GOAL)
  // 	camera_buttons_[previous_camera_mode_]->setEnabled(true);

  //     ROS_INFO("[DViz] Switching to automatic camera control.");
  //     view_manager->setCurrentViewControllerType("rviz/Orbit");
  //     view_manager->getCurrent()->subProp("Target Frame")->setValue("/map");
  //     view_manager->getCurrent()->subProp("Distance")->setValue(4.0);

  //     auto_camera_state_ = "normal";
  //     auto_camera_cached_yaw_ = 0.0;
  //   }

  //   geometry_msgs::Point midpoint;
  //   midpoint.x = (A.position.x + B.position.x)/2.0;
  //   midpoint.y = (A.position.y + B.position.y)/2.0;
  //   midpoint.z = (A.position.z + B.position.z)/2.0;

  //   // First focus camera to the appropriate position.
  //   view_manager->getCurrent()->subProp("Focal Point")->subProp("X")->setValue(midpoint.x);
  //   view_manager->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue(midpoint.y);
  //   view_manager->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue(midpoint.z);

  //   // Next choose the appropriate zoom
  //   Ogre::Camera *camera = view_manager->getCurrent()->getCamera();
  //   // Vertical field of view.
  //   float V = camera->getFOVy().valueRadians();
  //   //float H = camera->getFOVx().valueRadians();
  //   // Aspect ratio.
  //   float r = camera->getAspectRatio();
  //   // Horizontal field of view.
  //   float H = 2*std::atan(0.5*std::tan(V) * r);
  //   float th = std::min(H,V);
  //   double dx = B.position.x - A.position.x;
  //   double dy = B.position.y - A.position.y;
  //   double dz = B.position.z - A.position.z;
  //   double d = sqrt(dx*dx+dy*dy+dz*dz) + 4.0;
  //   double zoom = d/(2*tan(th/2.0));
  //   view_manager->getCurrent()->subProp("Distance")->setValue(zoom);

  //   //roll is 0 since we always want the camera straight up and down (with respect to gravity)
  //   // @todo the view controller does not allow us to set the roll.
  //   //view_manager->getCurrent()->subProp("Roll")->setValue(0.0);
  //   view_manager->getCurrent()->getCamera()->roll(Ogre::Radian(0.0));

  //   //choose the pitch and yaw to be on the highest point on the circle orthogonal to the line
  //   //this is computed by crossing the hand to goal vector with a vector that has no z component and is rotated 90 degress in xy
  //   double v1x = dx;
  //   double v1y = dy;
  //   double v1z = dz;
  //   double v2x = -v1y;
  //   double v2y = v1x;
  //   double v2z = 0;
  //   double v3x = v1y*v2z - v1z*v2y;
  //   double v3y = v1z*v2x - v1x*v2z;
  //   double v3z = v1x*v2y - v1y*v2x;

  //   double yaw_angle = atan2(v3y,v3x);
  //   double xy_length = sqrt(v3x*v3x+v3y*v3y);
  //   double pitch_angle = atan2(v3z, xy_length);

  //   //determine if we are in a special region on the sphere
  //   if(strcmp(auto_camera_state_.c_str(),"normal")==0){
  //     if(pitch_angle > 85.0*M_PI/180.0)
  // 	auto_camera_state_ = "north pole";
  //     if(pitch_angle < 15.0*M_PI/180.0)
  // 	auto_camera_state_ = "equator";

  //     view_manager->getCurrent()->subProp("Yaw")->setValue(yaw_angle);
  //     view_manager->getCurrent()->subProp("Pitch")->setValue(pitch_angle);
  //     auto_camera_cached_yaw_ = yaw_angle;
  //     auto_camera_cached_pitch_ = pitch_angle;
  //   }
  //   else if(strcmp(auto_camera_state_.c_str(),"north pole")==0){
  //     if(pitch_angle < 80.0*M_PI/180.0)
  // 	auto_camera_state_ = "normal";

  //     view_manager->getCurrent()->subProp("Yaw")->setValue(auto_camera_cached_yaw_);
  //     view_manager->getCurrent()->subProp("Pitch")->setValue(auto_camera_cached_pitch_);
  //   }
  //   else if(strcmp(auto_camera_state_.c_str(),"equator")==0){
  //     if(pitch_angle > 30.0*M_PI/180.0)
  // 	auto_camera_state_ = "normal";

  //     view_manager->getCurrent()->subProp("Yaw")->setValue(auto_camera_cached_yaw_);
  //     view_manager->getCurrent()->subProp("Pitch")->setValue(auto_camera_cached_pitch_);
  //   }
  //   else{
  //     ROS_ERROR("auto camera is in a bad state");
  //   }

  //   break;
  // }
  // case TOP_DOWN_FPS:
  // {
  //   if(previous_camera_mode_ != TOP_DOWN_FPS)
  //   {
  //     camera_buttons_[TOP_DOWN_FPS]->setEnabled(false);
  //     if(previous_camera_mode_ != GOAL)
  // 	camera_buttons_[previous_camera_mode_]->setEnabled(true);

  //     ROS_INFO("[DViz] Switching to top down/FPS camera control.");
  //   }

  //   if(top_down_fps_camera_mode_ == 0)
  //   {
  //     // Top Down camera mode.
  //     if(last_top_down_fps_camera_mode_ != 0)
  //     {
  // 	view_manager->setCurrentViewControllerType("rviz/Orbit");
  // 	view_manager->getCurrent()->subProp("Target Frame")->setValue("/map");

  // 	last_top_down_fps_camera_mode_ = 0;

  // 	view_manager->getCurrent()->installEventFilter(this);
  //     }

  //     view_manager->getCurrent()->subProp("Pitch")->setValue(M_PI/2.0);

  //     geometry_msgs::Point midpoint;
  //     // geometry_msgs::Pose base_pose = user_.getBasePose();
  //     geometry_msgs::Pose goal_pose = B;
  //     midpoint.x = (base_pose.position.x + goal_pose.position.x)/2.0;
  //     midpoint.y = (base_pose.position.y + goal_pose.position.y)/2.0;
  //     // midpoint.z = (base_pose.position.z + goal_pose.position.z)/2.0;
  //     midpoint.z = 0.0;

  //     // First focus camera to the appropriate position.
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("X")->setValue(midpoint.x);
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue(midpoint.y);
  //     view_manager->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue(midpoint.z);

  //     // Next choose the appropriate zoom
  //     Ogre::Camera *camera = view_manager->getCurrent()->getCamera();
  //     // Vertical field of view.
  //     float V = camera->getFOVy().valueRadians();
  //     // Aspect ratio.
  //     float r = camera->getAspectRatio();
  //     // Horizontal field of view.
  //     float H = 2*std::atan(0.5*std::tan(V) * r);
  //     float th = std::min(H, V);
  //     double dx = goal_pose.position.x - base_pose.position.x;
  //     double dy = goal_pose.position.y - base_pose.position.y;
  //     double dz = goal_pose.position.z - base_pose.position.z;
  //     double d = std::sqrt(dx*dx + dy*dy + dz*dz) + 2.0;
  //     double zoom = d/(2*tan(th/2.0));
  //     view_manager->getCurrent()->subProp("Distance")->setValue(zoom);

  //   }
  //   else if(top_down_fps_camera_mode_ == 1)
  //   {
  //     // FPS camera mode.
  //     if(last_top_down_fps_camera_mode_ != 1)
  //     {
  // 	view_manager->setCurrentViewControllerType("rviz/FPS");
  // 	view_manager->getCurrent()->subProp("Target Frame")->setValue("base_footprint");
  // 	view_manager->getCurrent()->subProp("Position")->subProp("X")->setValue(0.0);
  // 	view_manager->getCurrent()->subProp("Position")->subProp("Y")->setValue(0.0);
  // 	view_manager->getCurrent()->subProp("Position")->subProp("Z")->setValue(1.5);

  // 	last_top_down_fps_camera_mode_ = 1;

  // 	view_manager->getCurrent()->installEventFilter(this);
  //     }

  //     // geometry_msgs::Pose end_effector_pose = user_.getEndEffectorPoseInBase();
  //     // geometry_msgs::Pose base_pose = user_.getBasePose();

  //     double theta = std::atan2(end_effector_pose.position.y, end_effector_pose.position.x);
  //     double r = std::sqrt(std::pow(end_effector_pose.position.x, 2) + std::pow(end_effector_pose.position.y, 2));
  //     double pitch = M_PI/2.0 - std::atan2(r * std::cos(theta), 1.5 - end_effector_pose.position.z);
  //     view_manager->getCurrent()->subProp("Yaw")->setValue(tf::getYaw(base_pose.orientation) + theta);
  //     view_manager->getCurrent()->subProp("Pitch")->setValue(pitch);
  //   }
  //   else
  //   {
  //     ROS_ERROR("[DViz] Top Down/FPS camera invalid mode!");
  //   }

  //   break;
  // }
  // case GOAL:
  // {
  //   if(previous_camera_mode_ != GOAL)
  //   {
  //     camera_buttons_[previous_camera_mode_]->setEnabled(true);

  //     // Set the focal point of the camera to be the goal.
  //     ROS_INFO("[DViz] Switching to goal camera mode.");
  //     view_manager->setCurrentViewControllerType("rviz/Orbit");
  //     view_manager->getCurrent()->subProp("Target Frame")->setValue("map");
  //     view_manager->getCurrent()->subProp("Distance")->setValue(1.5);

  //     // hack
  //     last_top_down_fps_camera_mode_ = 2;
  //   }

  //   view_manager->getCurrent()->subProp("Focal Point")->subProp("X")->setValue(B.position.x);
  //   view_manager->getCurrent()->subProp("Focal Point")->subProp("Y")->setValue(B.position.y);
  //   view_manager->getCurrent()->subProp("Focal Point")->subProp("Z")->setValue(B.position.z);

  //   break;
  // }
  // default:
  //   break;
  // }

  // previous_camera_mode_ = camera_mode_;
}

void DemonstrationVisualizerClient::changeCameraMode(int mode)
{
  if(getState() == GRASP_SELECTION && camera_mode_ == GOAL)
  {
    QMessageBox box;

    box.setText("Cannot change the camera mode while selecting a grasp!");

    box.exec();

    return;
  }

  ROS_INFO("[DViz] Changing camera mode from %d to %d.", camera_mode_, mode);

  previous_camera_mode_ = (CameraMode)camera_mode_;

  camera_mode_ = (CameraMode)mode;
}

void DemonstrationVisualizerClient::pauseSimulator(bool later)
{
  QPixmap play_pause_pixmap(":/icons/play.png");
  QIcon play_pause_icon(play_pause_pixmap);
  play_pause_button_->setIcon(play_pause_icon);
  play_pause_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));

  playing_ = false;

  if(!user_.processCommand((later ? "pause_later" : "pause_now"), std::vector<std::string>()))
  {
    ROS_ERROR_STREAM("[DVizClient] Failed to pause the simulator " << (later ? "later" : "now")
		     << "!");
  }
}

void DemonstrationVisualizerClient::playSimulator()
{
  QPixmap play_pause_pixmap(":/icons/pause.png");
  QIcon play_pause_icon(play_pause_pixmap);
  play_pause_button_->setIcon(play_pause_icon);
  play_pause_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));

  playing_ = true;

  if(!user_.processCommand("play", std::vector<std::string>()))
  {
    ROS_ERROR("[DVizClient] Failed to play the simulator!");
  }
}

void DemonstrationVisualizerClient::toggleZMode()
{
  if(z_mode_)
    disableZMode();
  else
    enableZMode();
}

void DemonstrationVisualizerClient::enableZMode()
{
  ROS_INFO("[DViz] Enabling Z-mode.");
  z_mode_ = true;
  z_mode_button_->setText("Disable Z-Mode");
  std::vector<std::string> args;
  args.push_back(boost::lexical_cast<std::string>(Qt::Key_Z));
  args.push_back(boost::lexical_cast<std::string>(QEvent::KeyPress));
  if(!user_.processCommand("process_key", args))
  {
    ROS_ERROR("[DVizClient] Failed to process key event!");
  }
}

void DemonstrationVisualizerClient::disableZMode()
{
  ROS_INFO("[DViz] Disabling Z-mode.");
  z_mode_ = false;
  z_mode_button_->setText("Enable Z-Mode");
  std::vector<std::string> args;
  args.push_back(boost::lexical_cast<std::string>(Qt::Key_Z));
  args.push_back(boost::lexical_cast<std::string>(QEvent::KeyRelease));
  if(!user_.processCommand("process_key", args))
  {
    ROS_ERROR("[DVizClient] Failed to process key event!");
  }
}

void DemonstrationVisualizerClient::setFPSXOffset(int offset)
{
  if(camera_mode_ == FPS)
  {
    x_fps_offset_ = (double)offset/100;
  }
}

void DemonstrationVisualizerClient::setFPSZOffset(int offset)
{
  if(camera_mode_ == FPS)
  {
    z_fps_offset_ = (double)offset/100;
  }
}

void DemonstrationVisualizerClient::setGripperPosition(int position)
{
  double p = (double)position/1000.0;
  
  if(getState() == GRASP_SELECTION)
  {
    int current_goal = user_.getSceneManager()->getCurrentGoal();

    if(user_.getSceneManager()->getGoal(current_goal)->getType() != Goal::PICK_UP)
    {
      ROS_ERROR("[DViz] Goal of wrong type! (This should not occur.)");
      return;
    }

    PickUpGoal *goal = static_cast<PickUpGoal *>(user_.getSceneManager()->getGoal(current_goal));
    goal->setGripperJointPosition(p);
    // @TODO
    // user_.showInteractiveGripper(user_.getSceneManager()->getCurrentGoal());
  }
}

void DemonstrationVisualizerClient::toggleStartStop()
{
  if(started_)
  {
    started_ = false;
    endBasicMode();
  }
  else
  {
    started_ = true;
    startBasicMode();
  }
}

void DemonstrationVisualizerClient::togglePlayPause()
{
  if(playing_)
  {
    pauseSimulator();
  }
  else
  {
    playSimulator();
  }
}

void DemonstrationVisualizerClient::toggleGripperOrientationMode()
{
  bool disable = false;
  if(gripper_orientation_mode_)
  {
    gripper_orientation_mode_ = false;
    QPixmap gripper_orientation_pixmap(":/icons/full_control.png");
    QIcon gripper_orientation_icon(gripper_orientation_pixmap);
    gripper_orientation_button_->setIcon(gripper_orientation_icon);
    gripper_orientation_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
    disable = true;
  }
  else
  {
    QPixmap gripper_orientation_pixmap(":/icons/simple_control.png");
    QIcon gripper_orientation_icon(gripper_orientation_pixmap);
    gripper_orientation_button_->setIcon(gripper_orientation_icon);
    gripper_orientation_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
    gripper_orientation_mode_ = true;
    disable = false;
  }
  std::vector<std::string> args;
  args.push_back((disable ? "true" : "false"));
  if(!user_.processCommand("basic_gripper", args))
  {
    ROS_ERROR("[DVizClient] Failed to toggle gripper controls!");
  }
}

void DemonstrationVisualizerClient::toggleCollisions()
{
  if(collisions_mode_)
  {
    // Turn off collision detection (ignore = true).
    collisions_mode_ = false;
    QPixmap toggle_collisions_pixmap(":/icons/enable_collisions.png");
    QIcon toggle_collisions_icon(toggle_collisions_pixmap);
    toggle_collisions_button_->setIcon(toggle_collisions_icon);
    toggle_collisions_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
    // @TODO
    // user_.setIgnoreCollisions(true);
  }
  else
  {
    // Turn on collision detection (ignore = false).
    collisions_mode_ = true;
    QPixmap toggle_collisions_pixmap(":/icons/disable2_collisions.png");
    QIcon toggle_collisions_icon(toggle_collisions_pixmap);
    toggle_collisions_button_->setIcon(toggle_collisions_icon);
    toggle_collisions_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
    // @TODO
    // user_.setIgnoreCollisions(false);
  }
}

void DemonstrationVisualizerClient::acceptChangeGrasp()
{
  if(accepted_grasp_)
  {
    QPixmap accept_change_grasp_pixmap(":/icons/accept_grasp.png");
    QIcon accept_change_grasp_icon(accept_change_grasp_pixmap);
    accept_change_grasp_button_->setIcon(accept_change_grasp_icon);
    accept_change_grasp_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));
    
    beginGraspSelection();
    accepted_grasp_ = false;
  }
  else
  {
    QPixmap accept_change_grasp_pixmap(":/icons/change_grasp.png");
    QIcon accept_change_grasp_icon(accept_change_grasp_pixmap);
    accept_change_grasp_button_->setIcon(accept_change_grasp_icon);
    accept_change_grasp_button_->setIconSize(QSize(ICON_SIZE, ICON_SIZE));

    endGraspSelection();
    accepted_grasp_ = true;
  }
}

void DemonstrationVisualizerClient::beginGraspSelection()
{
  changeState(GRASP_SELECTION);

  pauseSimulator();
  // @TODO
  // user_.disableRobotMarkerControl();

  accept_change_grasp_button_->setEnabled(true);
  grasp_distance_slider_->setEnabled(true);
  gripper_position_slider_->setEnabled(true);

  // Set the distance slider to the goal's current distance.
  int current_goal = user_.getSceneManager()->getCurrentGoal();
  PickUpGoal *goal = static_cast<PickUpGoal *>(user_.getSceneManager()->getGoal(current_goal));
  grasp_distance_slider_->setValue(static_cast<int>(goal->getGraspDistance()*100.0));
  gripper_position_slider_->setValue(static_cast<int>(goal->getGripperJointPosition()*1000.0));

  camera_before_grasp_ = camera_mode_;
  changeCameraMode(GOAL);
  // @TODO
  // user_.showInteractiveGripper(current_goal);
}

void DemonstrationVisualizerClient::endGraspSelection()
{
  changeState(NORMAL);

  int current_goal = user_.getSceneManager()->getCurrentGoal();
  std::stringstream s; 
  s << "grasp_marker_goal_" << current_goal;
  // @TODO
  // user_.getInteractiveMarkerServer()->erase(s.str());
  // user_.getInteractiveMarkerServer()->applyChanges();
  PickUpGoal *goal = static_cast<PickUpGoal *>(user_.getSceneManager()->getGoal(current_goal));
  goal->setGraspDone(true);
  user_.getSceneManager()->setGoalsChanged();
  changeCameraMode(camera_before_grasp_);
  grasp_distance_slider_->setEnabled(false);
  gripper_position_slider_->setEnabled(false);

  playSimulator();
  // @TODO
  // user_.enableRobotMarkerControl();
}

void DemonstrationVisualizerClient::setGraspDistance(int value)
{
  double distance = (double)value/100.0;

  int current_goal = user_.getSceneManager()->getCurrentGoal();
  PickUpGoal *goal = static_cast<PickUpGoal *>(user_.getSceneManager()->getGoal(current_goal));
  goal->setGraspDistance(distance);
  
  // Redraw the interactive marker on the gripper to reflect the change in distance.
  // @TODO
  // user_.showInteractiveGripper(current_goal);
}

} // namespace demonstration_visualizer
