#include <dviz_core/demonstration_scene_manager.h>

namespace demonstration_visualizer
{

const std::string DemonstrationSceneManager::GOAL_MARKER_NAMESPACE = "dviz_goal";

DemonstrationSceneManager::DemonstrationSceneManager(
    PViz *pviz,
    interactive_markers::InteractiveMarkerServer *int_marker_server,
    ObjectManager* object_manager,
    int user_id
  )
  : user_id_(user_id), goals_changed_(false), edit_goals_mode_(false),
    edit_meshes_mode_(false), int_marker_server_(int_marker_server), current_goal_(-1),
    object_manager_(object_manager), pviz_(pviz)
{
  goal_feedback_ = boost::bind(&DemonstrationSceneManager::processGoalFeedback,
			       this,
			       _1);

  mesh_feedback_ = boost::bind(&DemonstrationSceneManager::processMeshFeedback,
			       this,
			       _1);

  ros::NodeHandle nh;

  marker_pub_ = nh.advertise<visualization_msgs::Marker>(resolveName("visualization_marker", user_id_), 0);

  initial_robot_pose_.position.x = 0;
  initial_robot_pose_.position.y = 0;
  initial_robot_pose_.position.z = 0;
  initial_robot_pose_.orientation.w = 1;

  initial_torso_position_ = 0.3;
}

DemonstrationSceneManager::~DemonstrationSceneManager()
{
  for(int i = 0; i < goals_.size(); ++i) 
  {
    if(goals_[i] != 0)
    {
      delete goals_[i];
      goals_[i] = 0;
    }
  }
}

void DemonstrationSceneManager::updateScene()
{
  // First, update the goals
  if(goals_.size() > 0 && !taskDone() && goalsChanged())
  {
    // Draw each of the goals in the current task
    if(editGoalsMode())
    {
      std::vector<Goal *>::iterator it;
      for(it = goals_.begin(); it != goals_.end(); ++it)
      {
	drawGoal(*it, true);
      }
    }
    else // Otherwise, just draw the current goal
    {
      // Clear the existing goal markers and interactive markers, and just draw the 
      // current goal
      std::vector<Goal *>::iterator it;
      for(it = goals_.begin(); it != goals_.end(); ++it)
      {
	if((*it)->getGoalNumber() == current_goal_)
	{
	  //ROS_INFO("[SceneManager%d] Drawing goal %d", user_id_, (*it)->getGoalNumber());
	  drawGoal(*it, false);
	}
	else
	{
	  //ROS_INFO("[SceneManager%d] Hiding goal %d", user_id_, (*it)->getGoalNumber());
	  hideGoal(*it);
	}
      }
    }
    
    setGoalsChanged(false);
  }

  // Second, update the positions of the objects
  std::vector<visualization_msgs::Marker> meshes = object_manager_->getMovedMarkers();  
  if(meshes.size() > 0)
  {
    if(editMeshesMode())
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	it->header.frame_id = resolveName("map", user_id_);
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::DELETE;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->mesh_use_embedded_materials = true;

	// First remove the old markers
	marker_pub_.publish(*it);

	// Then add the markers again, but this time with interactive markers
	it->action = visualization_msgs::Marker::ADD;
	visualizeMesh(*it, true);
      }
    }
    else
    {
      if(int_marker_server_ == 0)
      {
	ROS_WARN("[SceneManager%d] Interactive marker server is null.", user_id_);
	return;
      }
      
      std::vector<visualization_msgs::Marker>::iterator it;
      // For each mesh, first remove all the interactive markers from the meshes.
      // Then, re-visualize each marker without an attached interactive marker.
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	std::stringstream int_marker_name;
	int_marker_name << "mesh_marker_" << it->id;

	if(!int_marker_server_->erase(int_marker_name.str()))
	{
	  ROS_ERROR("[SceneManager%d] Failed to remove interactive marker on mesh %d!", user_id_, it->id);
	}
	int_marker_server_->applyChanges();

	it->header.frame_id = resolveName("map", user_id_);
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::ADD;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->color.r = it->color.g = it->color.b = it->color.a = 0;
	it->mesh_use_embedded_materials = true;

	marker_pub_.publish(*it);
      }	
    }
  }
}

int DemonstrationSceneManager::loadScene(const std::string &filename)
{
  ROS_INFO("[SceneManager%d] Loading scene from \"%s\"", user_id_, filename.c_str());

  ros::Time t_start_load = ros::Time::now();

  // Clear existing meshes.
  if(int_marker_server_ != 0)
  {
    std::vector<visualization_msgs::Marker> meshes = object_manager_->getMarkers();
    std::vector<visualization_msgs::Marker>::iterator it;
    for(it = meshes.begin(); it != meshes.end(); ++it)
    {
      std::stringstream int_marker_name;
      int_marker_name << "mesh_marker_" << it->id;

      if(!int_marker_server_->erase(int_marker_name.str()))
      {
	ROS_ERROR("[SceneManager%d] Failed to remove interactive marker on mesh %d!", user_id_, it->id);
      }
      int_marker_server_->applyChanges();

      it->header.frame_id = resolveName("map", user_id_);
      it->header.stamp = ros::Time();
      it->action = visualization_msgs::Marker::DELETE;
      it->type = visualization_msgs::Marker::MESH_RESOURCE;
      it->color.r = it->color.g = it->color.b = it->color.a = 0;
      it->mesh_use_embedded_materials = true;

      marker_pub_.publish(*it);
    }
  }
  else
  {
    ROS_WARN("[SceneManager%d] Interactive marker server is null.", user_id_);
  }

  object_manager_->clearObjects();

  // Load the demonstration scene from the specified file.
  int max_mesh_id = -1;
  TiXmlDocument doc(filename.c_str());
  if(!doc.LoadFile())
  {
    ROS_ERROR("[SceneManager%d] Failed to load file \"%s\"!", user_id_, filename.c_str());
    return max_mesh_id;
  }

  TiXmlHandle doc_handle(&doc);
  TiXmlElement *element;
  TiXmlHandle root_handle(0);

  element = doc_handle.FirstChildElement().Element();
  if(!element)
  {
    // @todo error
    return max_mesh_id;
  }

  root_handle = TiXmlHandle(element);

  ROS_INFO("Reading %s...", element->Value());

  // Read the path to the collision model files.
  element = root_handle.FirstChild().Element();
  std::string package_path = ros::package::getPath("dviz_core");
  if(element == NULL)
  {
    ROS_ERROR("This is broken. XML parsing of the scene file went wrong. (element = NULL)");
    return -1;
  }
  std::string collision_model_path = std::string(element->Attribute("path"));
  ROS_INFO("Collision model path is %s", collision_model_path.c_str());

  // Read size of the environment
  element = element->NextSiblingElement();
  double origin_x, origin_y, origin_z, size_x, size_y, size_z;
  if(element->QueryDoubleAttribute("origin_x", &origin_x) != TIXML_SUCCESS){
    ROS_ERROR("origin_x is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("origin_y", &origin_y) != TIXML_SUCCESS){
    ROS_ERROR("origin_y is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("origin_z", &origin_z) != TIXML_SUCCESS){
    ROS_ERROR("origin_z is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("size_x", &size_x) != TIXML_SUCCESS){
    ROS_ERROR("size_x is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("size_y", &size_y) != TIXML_SUCCESS){
    ROS_ERROR("size_y is missing");
    return -1;
  }
  if(element->QueryDoubleAttribute("size_z", &size_z) != TIXML_SUCCESS){
    ROS_ERROR("size_z is missing");
    return -1;
  }
  vector<double> origin;
  origin.push_back(origin_x);
  origin.push_back(origin_y);
  origin.push_back(origin_z);
  vector<double> size;
  size.push_back(size_x);
  size.push_back(size_y);
  size.push_back(size_z);

  if(!object_manager_->initializeCollisionChecker(size, origin))
    return false;

  // Read the filename containing the occupied voxels in the scene
  TiXmlElement *voxel_element = element->NextSiblingElement("object_voxels");
  std::string vfile;
  if(voxel_element != NULL){
    ROS_INFO("Found 'object_voxels' element in scene.");
    if(voxel_element->QueryStringAttribute("file", &vfile) != TIXML_SUCCESS)
      ROS_ERROR("'object_voxels' file is missing.");
    else{
      if(!object_manager_->addObjectsFromOccupiedVoxelsFile(vfile))
        return false;
      else
        ROS_INFO("Successfully added object voxels from file.");
    } 
  }

  // Read the initial configuration of the robot.
  TiXmlElement *initial_element = element->NextSiblingElement("initial_configuration");
  if(initial_element == NULL)
  {
    ROS_WARN("[SceneManager%d] Found no initial robot configuration in the scene file (default = origin)!",
      user_id_);
    // If this element does not exist, assume the initial pose of the robot is 
    // at the origin.
    initial_robot_pose_.position.x = 0;
    initial_robot_pose_.position.y = 0;
    initial_robot_pose_.position.z = 0;
    initial_robot_pose_.orientation.x = 0;
    initial_robot_pose_.orientation.y = 0;
    initial_robot_pose_.orientation.z = 0;
    initial_robot_pose_.orientation.w = 1;
  }
  else
  {
    if(initial_element->QueryDoubleAttribute("x", &initial_robot_pose_.position.x) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager%d] Could not read the initial x position of the robot!", user_id_);
      return -1;
    }

    if(initial_element->QueryDoubleAttribute("y", &initial_robot_pose_.position.y) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager%d] Could not read the initial y position of the robot!", user_id_);
      return -1;
    }
    
    double yaw = 0;
    if(initial_element->QueryDoubleAttribute("yaw", &yaw) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager%d] Could not read the initial yaw of the robot!", user_id_);
      return -1;
    }
    initial_robot_pose_.orientation = tf::createQuaternionMsgFromYaw(yaw);

    if(initial_element->QueryDoubleAttribute("torso_position", &initial_torso_position_) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager%d] Could not read the initial torso position of the robot!", user_id_);
      return -1;
    }
  }

  // Read each mesh.
  element = element->NextSiblingElement("mesh");
  visualization_msgs::Marker mesh_marker;

  // for(element; element; element = element->NextSiblingElement("mesh"))
  while(element)
  {
    if(element->QueryIntAttribute("id", &mesh_marker.id) != TIXML_SUCCESS)
    {
      ROS_ERROR("id is missing");
      return -1;
    }
    std::string label = std::string(element->Attribute("label"));
    if(element->QueryDoubleAttribute("position_x", &mesh_marker.pose.position.x) != TIXML_SUCCESS)
    {
      ROS_ERROR("position_x is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("position_y", &mesh_marker.pose.position.y) != TIXML_SUCCESS)
    {
      ROS_ERROR("position_y is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("position_z", &mesh_marker.pose.position.z) != TIXML_SUCCESS)
    {
      ROS_ERROR("position_z is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("orientation_x", &mesh_marker.pose.orientation.x) != TIXML_SUCCESS)
    {
      ROS_ERROR("orientation_x is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("orientation_y", &mesh_marker.pose.orientation.y) != TIXML_SUCCESS)
    {
      ROS_ERROR("orientation_y is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("orientation_z", &mesh_marker.pose.orientation.z) != TIXML_SUCCESS)
    {
      ROS_ERROR("orientation_z is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("orientation_w", &mesh_marker.pose.orientation.w) != TIXML_SUCCESS)
    {
      ROS_ERROR("orientation_w is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("scale_x", &mesh_marker.scale.x) != TIXML_SUCCESS)
    {
      ROS_ERROR("scale_x is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("scale_y", &mesh_marker.scale.y) != TIXML_SUCCESS)
    {
      ROS_ERROR("scale_y is missing");
      return -1;
    }
    if(element->QueryDoubleAttribute("scale_z", &mesh_marker.scale.z) != TIXML_SUCCESS)
    {
      ROS_ERROR("scale_z is missing");
      return -1;
    }

    mesh_marker.header.frame_id = resolveName("map", user_id_);
    mesh_marker.header.stamp = ros::Time();
    mesh_marker.ns = "demonstration_visualizer";
    mesh_marker.action = visualization_msgs::Marker::ADD;
    mesh_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    mesh_marker.color.r = mesh_marker.color.g = mesh_marker.color.b = mesh_marker.color.a = 0;
    mesh_marker.mesh_use_embedded_materials = true;

    std::stringstream collision_model_file;
    collision_model_file << package_path << collision_model_path << "/" << label << ".xml";

    int movable;
    if(element->QueryIntAttribute("movable", &movable) != TIXML_SUCCESS){
      ROS_ERROR("movable is missing");
      return -1;
    }
    object_manager_->addObjectFromFile(mesh_marker,
				       collision_model_file.str(),
				       movable);

    visualizeMesh(mesh_marker, editMeshesMode());

    if(mesh_marker.id > max_mesh_id)
      max_mesh_id = mesh_marker.id;

    element = element->NextSiblingElement("mesh");
  }

  ROS_INFO("[dsm] Finished getting all the elements...now setting the meshes.");

  ROS_INFO("[dsm] Read %d meshes.", (int)object_manager_->getNumObjects());

  ros::Time t_end_load = ros::Time::now();
  ROS_WARN("[dsm] Loading the scene took %0.3fsec", ros::Duration(t_end_load-t_start_load).toSec());

  object_manager_->writeObjectsToOccupiedVoxelsFile("/tmp/object_voxel2.csv");

  // object_manager_->visualizeObjectCollisionModels();

  return max_mesh_id;
}

void DemonstrationSceneManager::saveScene(const std::string &filename)
{
  TiXmlDocument doc;
  TiXmlElement *root = new TiXmlElement("scene");
  doc.LinkEndChild(root);

  // @todo for now, just set the collision models directory to the default.
  // idea: we could create a directory with the same name as the scene file
  //       to store collision models specific to that scene, but this might
  //       make it harder to have shared collision model files.
  TiXmlElement *collision_models = new TiXmlElement("collision_models");
  collision_models->SetAttribute("path", "/collision_models");
  root->LinkEndChild(collision_models);

  // Store the origin and dimensions of the bounding box.
  TiXmlElement *size = new TiXmlElement("size");
  std::vector<double> dimensions = object_manager_->getBoundingBoxDimensions();
  std::vector<double> origin = object_manager_->getBoundingBoxOrigin();
  size->SetDoubleAttribute("origin_x", origin[0]);
  size->SetDoubleAttribute("origin_y", origin[1]);
  size->SetDoubleAttribute("origin_z", origin[2]);
  size->SetDoubleAttribute("size_x", dimensions[0]);
  size->SetDoubleAttribute("size_y", dimensions[1]);
  size->SetDoubleAttribute("size_z", dimensions[2]);
  root->LinkEndChild(size);

  TiXmlElement *initial_configuration = new TiXmlElement("initial_configuration");
  // @todo allow the user to edit the initial configuration. For now, set it to be
  // the origin.
  initial_configuration->SetDoubleAttribute("x", 0);
  initial_configuration->SetDoubleAttribute("y", 0);
  initial_configuration->SetDoubleAttribute("yaw", 0);
  initial_configuration->SetDoubleAttribute("torso_position", 0);

  // For each object, add the mesh to the scene file and create a new 
  // collision model file for it.
  std::vector<Object> objects = object_manager_->getObjects();
  std::vector<Object>::iterator it;
  for(it = objects.begin(); it != objects.end(); ++it)
  {
    TiXmlElement *mesh = new TiXmlElement("mesh");

    mesh->SetAttribute("id", it->mesh_marker_.id);
    mesh->SetAttribute("label", it->label);
    ROS_INFO("label = %s", it->label.c_str());
    mesh->SetDoubleAttribute("position_x", it->mesh_marker_.pose.position.x);
    mesh->SetDoubleAttribute("position_y", it->mesh_marker_.pose.position.y);
    mesh->SetDoubleAttribute("position_z", it->mesh_marker_.pose.position.z);
    mesh->SetDoubleAttribute("orientation_x", it->mesh_marker_.pose.orientation.x);
    mesh->SetDoubleAttribute("orientation_y", it->mesh_marker_.pose.orientation.y);
    mesh->SetDoubleAttribute("orientation_z", it->mesh_marker_.pose.orientation.z);
    mesh->SetDoubleAttribute("orientation_w", it->mesh_marker_.pose.orientation.w);
    mesh->SetDoubleAttribute("scale_x", it->mesh_marker_.scale.x);
    mesh->SetDoubleAttribute("scale_y", it->mesh_marker_.scale.y);
    mesh->SetDoubleAttribute("scale_z", it->mesh_marker_.scale.z);
    mesh->SetAttribute("movable", static_cast<int>(it->movable));
  
    root->LinkEndChild(mesh);
    
    // Add the collision model file, if it does not already exist.
    std::stringstream ss;
    ss << ros::package::getPath("dviz_core") << "/collision_models/" << it->label << ".xml";
    ROS_INFO("[SceneManager%d] Checking if %s exists...", user_id_, ss.str().c_str());
    if(!boost::filesystem::exists(ss.str()))
    {
      ROS_INFO("...it doesn't.");

      TiXmlDocument cm_file;
      TiXmlElement *cm_file_root = new TiXmlElement("collision_model");
      cm_file.LinkEndChild(cm_file_root);

      TiXmlElement *object = new TiXmlElement("object");
      object->SetAttribute("label", it->label);
      object->SetAttribute("mesh_resource", it->mesh_marker_.mesh_resource);
      cm_file_root->LinkEndChild(object);

      // Add collision spheres if the object is movable.
      if(it->movable)
      {
	std::vector<pr2_collision_checker::Sphere> spheres = it->group_.spheres;

	std::vector<pr2_collision_checker::Sphere>::iterator sit;
	for(sit = spheres.begin(); sit != spheres.end(); ++sit)
	{
	  TiXmlElement *sphere = new TiXmlElement("sphere");
	  sphere->SetAttribute("id", sit->name);
	  sphere->SetDoubleAttribute("x", sit->v.x());
	  sphere->SetDoubleAttribute("y", sit->v.y());
	  sphere->SetDoubleAttribute("z", sit->v.z());
	  sphere->SetDoubleAttribute("radius", sit->radius);
	  cm_file_root->LinkEndChild(sphere);
	}
      }

      cm_file.SaveFile(ss.str().c_str());
    }
    else
    {
      ROS_WARN("[SceneManager%d] Collision model file \"%s\" already exists!", user_id_, ss.str().c_str());
    }
  }

  doc.SaveFile(filename.c_str());
}

bool DemonstrationSceneManager::loadTask(const std::string &filename, bool randomize, int max_goals)
{
  // Clear the existing task
  for(int i = 0; i < int(goals_.size()); ++i)
  {
    delete goals_[i];
    goals_[i] = 0;
  }
  goals_.clear();
  setGoalsChanged(true);

  // Load the demonstration scene from the specified file
  TiXmlDocument doc(filename.c_str());
  if(!doc.LoadFile())
  {
    ROS_ERROR("[SceneManager%d] Failed to load file \"%s\"!", user_id_, filename.c_str());
    return false;
  }

  TiXmlHandle doc_handle(&doc);
  TiXmlElement *element;
  TiXmlHandle root_handle(0);

  element = doc_handle.FirstChildElement().Element();
  if(!element)
  {
    // @todo error
    return false;
  }

  root_handle = TiXmlHandle(element);

  ROS_INFO("Reading %s...", element->Value());

  // @todo for now, we assume the user has loaded the appropriate scene file, but 
  // in the future there should be a tag at the top of the file:
  // <scene_file path="..." />
  // where path is relative to the package://dviz_core path
  TiXmlElement *scene_file = root_handle.FirstChild("scene_file").Element();
  if(scene_file == NULL)
  {
    ROS_WARN("[SceneManager%d] No scene file specified for the given task.", user_id_);
  }
  else
  {
    ROS_INFO("val = %s", scene_file->Value());
    std::string scene_filename = "";
    if(scene_file->QueryStringAttribute("path", &scene_filename) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager%d] Failed to read scene file path for this task!", user_id_);
      return false;
    }

    if(loadScene(scene_filename) < 0)
    {
      ROS_ERROR("[SceneManager%d] Failed to load scene associated with this task!", user_id_);
      return false;
    }
  }

  // There is also an optional task info tag specified at the top of the task file; right now, this only contains the name of the task
  TiXmlElement *task_info = root_handle.FirstChild("task_info").Element();
  if(task_info == NULL)
  {
    ROS_WARN("[SceneManager%d] No task info specified for the given task.", user_id_);
  }
  else
  {
    std::string task_name = "";
    if(task_info->QueryStringAttribute("name", &task_name) != TIXML_SUCCESS)
    {
      ROS_WARN("[SceneManager%d] No task name specified.", user_id_);
    }
    else
    {
      ROS_INFO("[SceneManager%d] Loading task \"%s\".", user_id_, task_name.c_str());
      task_name_ = task_name;
    }
  }

  // Read each goal
  element = root_handle.FirstChild("goal").Element();

  int goal_type = 0;
  int goal_number = 0;
  std::string goal_description = "";

  // for(element; element; element = element->NextSiblingElement())
  while(element)
  {
    if(element->QueryIntAttribute("type", &goal_type) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager%d] Failed to read goal type!", user_id_);
      return false;
    }
    if(element->QueryIntAttribute("number", &goal_number) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager%d] Failed to read goal number!", user_id_);
      return false;
    }
    if(element->QueryStringAttribute("desc", &goal_description) != TIXML_SUCCESS)
    {
      ROS_ERROR("[SceneManager%d] Failed to read goal description!", user_id_);
      return false;
    }

    switch(goal_type)
    {
    case Goal::PICK_UP:
    {
      PickUpGoal *goal = new PickUpGoal(goal_number, goal_description);

      int object_id = 0;
      if(element->QueryIntAttribute("object_id", &object_id) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read object ID!", user_id_);
	delete goal;
	return false;
      }

      // (Optional) For pick up goals, one can specify a camera position in polar coordinates,
      // centered around the pose of the object that is to be picked up.
      // In particular, the camera position is specified by a zenith angle phi
      // (i.e., the angle from the z-axis), an azimuth angle theta (i.e. the
      // angle from the x-axis around the z-axis), and a radius from the center position.
      double phi = M_PI / 4.0, theta = 0.0, radius = 1.0;
      if(element->QueryDoubleAttribute("camera_phi", &phi) != TIXML_SUCCESS)
      {
	ROS_WARN("[SceneManager%d] No camera azimuth angle specified", user_id_);
      }
      if(element->QueryDoubleAttribute("camera_theta", &theta) != TIXML_SUCCESS)
      {
	ROS_WARN("[SceneManager%d] No camera zenith angle specified", user_id_);
      }
      if(element->QueryDoubleAttribute("camera_radius", &radius) != TIXML_SUCCESS)
      {
	ROS_WARN("[SceneManager%d] No camera radius specified", user_id_);
      }

      geometry_msgs::Pose object_pose = object_manager_->getMarker(object_id).pose;
      goal->setGraspPose(object_pose);
      goal->setInitialObjectPose(object_pose);
      goal->setObjectID(object_id);

      goal->setCameraPhi(phi);
      goal->setCameraTheta(theta);
      goal->setCameraRadius(radius);

      goals_.push_back(goal);

      break;
    }
    case Goal::PLACE:
    {
      PlaceGoal *goal = new PlaceGoal(goal_number, goal_description);

      int object_id = 0;
      if(element->QueryIntAttribute("object_id", &object_id) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read object ID!", user_id_);
	delete goal;
	return false;
      }
      goal->setObjectID(object_id);

      int ignore_yaw = 0;
      if(element->QueryIntAttribute("ignore_yaw", &ignore_yaw) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read ignore yaw!", user_id_);
	// return false;
      }
      goal->setIgnoreYaw((bool)ignore_yaw);

      geometry_msgs::Pose place_pose;
      if(element->QueryDoubleAttribute("position_x", &place_pose.position.x) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read place position x!", user_id_);
	delete goal;
	return false;	  
      }
      if(element->QueryDoubleAttribute("position_y", &place_pose.position.y) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read place position y!", user_id_);
	delete goal;
	return false;	  
      }
      if(element->QueryDoubleAttribute("position_z", &place_pose.position.z) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read place position z!", user_id_);
	delete goal;
	return false;	  
      }
      if(element->QueryDoubleAttribute("orientation_x", &place_pose.orientation.x) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read place orientation x!", user_id_);
	delete goal;
	return false;	  
      }
      if(element->QueryDoubleAttribute("orientation_y", &place_pose.orientation.y) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read place orientation y!", user_id_);
	delete goal;
	return false;	  
      }
      if(element->QueryDoubleAttribute("orientation_z", &place_pose.orientation.z) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read place orientation z!", user_id_);
	delete goal;
	return false;	  
      }
      if(element->QueryDoubleAttribute("orientation_w", &place_pose.orientation.w) != TIXML_SUCCESS)
      {
	ROS_ERROR("[SceneManager%d] Failed to read place orientation w!", user_id_);
	delete goal;
	return false;	  
      }
      goal->setPlacePose(place_pose);

      goals_.push_back(goal);

      break;
    }
    default:
      break;
    }

    element = element->NextSiblingElement();
  }

  ROS_INFO("[SceneManager%d] Read %d goals", user_id_, (int)goals_.size());

  for(int i = 0; i < int(goals_.size()); ++i)
  {
    ROS_INFO_STREAM(goals_[i]->toString());
  }

  if(randomize)
    randomizePickAndPlaceTask();

  // If the max number of goals is greater than the number loaded, remove the last 
  // tasks (there must be at least one goal)
  if(max_goals > 0 && goals_.size() > max_goals)
  {
    int must_delete = goals_.size() - max_goals;
    ROS_INFO("[SceneManager%d] Removing %d goals", user_id_, must_delete);
    for(int i = goals_.size()-1; i >= must_delete; --i)
    {
      delete goals_[i];
      goals_[i] = 0;
      goals_.pop_back();
    }
  }

  if(getNumGoals() > 0)
    setCurrentGoal(0);

  return true;
}

void DemonstrationSceneManager::saveTask(const std::string &filename)
{
  TiXmlDocument doc;
  TiXmlElement *root = new TiXmlElement("task");
  doc.LinkEndChild(root);

  // Add each goal to the task description.
  std::vector<Goal *>::iterator it;
  for(it = goals_.begin(); it != goals_.end(); ++it)
  {
    TiXmlElement *goal = new TiXmlElement("goal");

    Goal::GoalType goal_type = (*it)->getType();
    switch(goal_type)
    {
    case Goal::PICK_UP:
    {
      PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(*it);

      goal->SetAttribute("number", pick_up_goal->getGoalNumber());
      goal->SetAttribute("type", (int)Goal::PICK_UP);
      goal->SetAttribute("desc", pick_up_goal->getDescription());
      goal->SetAttribute("object_id", pick_up_goal->getObjectID());
  
      break;
    }
    case Goal::PLACE:
    {
      PlaceGoal *place_goal = static_cast<PlaceGoal *>(*it);

      goal->SetAttribute("number", place_goal->getGoalNumber());
      goal->SetAttribute("type", (int)Goal::PLACE);
      goal->SetAttribute("desc", place_goal->getDescription());
      goal->SetAttribute("object_id", place_goal->getObjectID());
      goal->SetAttribute("ignore_yaw", (int)place_goal->ignoreYaw());
      goal->SetDoubleAttribute("position_x", place_goal->getPlacePose().position.x);
      goal->SetDoubleAttribute("position_y", place_goal->getPlacePose().position.y);
      goal->SetDoubleAttribute("position_z", place_goal->getPlacePose().position.z);
      goal->SetDoubleAttribute("orientation_x", place_goal->getPlacePose().orientation.x);
      goal->SetDoubleAttribute("orientation_y", place_goal->getPlacePose().orientation.y);
      goal->SetDoubleAttribute("orientation_z", place_goal->getPlacePose().orientation.z);
      goal->SetDoubleAttribute("orientation_w", place_goal->getPlacePose().orientation.w);

      break;
    }
    default:
      break;
    }

    root->LinkEndChild(goal);
  }

  doc.SaveFile(filename.c_str());
}

void DemonstrationSceneManager::resetTask()
{
  if(goals_.size() > 0)
    current_goal_ = 0;
  else
    current_goal_ = -1;

  std::vector<Goal *>::iterator it;
  for(it = goals_.begin(); it != goals_.end(); ++it)
  {
    if((*it)->getType() == Goal::PICK_UP)
    {
      PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(*it);
      geometry_msgs::Pose initial_object_pose = pick_up_goal->getInitialObjectPose();
      // Reset the grasp pose to the default.
      pick_up_goal->setGraspPose(initial_object_pose);
      //pick_up_goal->setGraspDistance(0.25);
      // Reset the object to its initial pose.
      object_manager_->moveObject(pick_up_goal->getObjectID(), initial_object_pose);
    }
  }
  
  setGoalsChanged();
}

void DemonstrationSceneManager::randomizePickAndPlaceTask()
{
  if(goals_.size() > 0)
  {
    // assert(goals_.size() % 2 == 0)
    std::vector<Goal *> random_goals(goals_.size(), 0);
    int goal_pairs = goals_.size()/2;
    std::vector<int> ordering;
    for(int i = 0; i < goal_pairs; ++i)
    {
      ordering.push_back(i);
    }
    std::srand(unsigned(std::time(0)));
    std::random_shuffle(ordering.begin(), ordering.end());
    ROS_INFO("rg size: %d", random_goals.size());
    // for(int i = 0; i < ordering.size(); ++i)
    // {
    //   std::cout << ordering[i] << " ";
    // }
    // std::cout << std::endl;
    // ROS_DEBUG("[SceneManager%d] New task ordering:", user_id_);
    for(int j = 0; j < goal_pairs; ++j)
    {
      goals_[2*j]->setGoalNumber(2*ordering[j]);
      goals_[2*j+1]->setGoalNumber(2*ordering[j]+1);
      random_goals[2*ordering[j]] = goals_[2*j];
      random_goals[2*ordering[j]+1] = goals_[2*j+1];
    }
    // for(std::vector<Goal *>::iterator it = random_goals.begin(); it != random_goals.end(); ++it)
    // {
    //   ROS_INFO("\t %s", (*it)->toString().c_str());
    // }

    goals_ = random_goals;
    ROS_DEBUG("[SceneManager%d] Task has been randomized", user_id_);
  }
  else
  {
    ROS_ERROR("[SceneManager%d] No task loaded", user_id_);
  }
}

void DemonstrationSceneManager::addMeshFromFile(const std::string &filename, 
						int mesh_id, 
						const std::string &label,
                                                bool movable,
                                                bool attach_interactive_marker)
{
  // Spawn the mesh at the origin
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header.frame_id = resolveName("map", user_id_);
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

  visualization_msgs::Marker marker = makeMeshMarker(filename,
						     "demonstration_visualizer",
						     mesh_id,
						     pose_stamped,
						     scale,
						     0.0,
						     true);

  addMesh(marker, attach_interactive_marker, label, movable);
}

void DemonstrationSceneManager::addMesh(const visualization_msgs::Marker &marker, 
					bool attach_interactive_marker,
					const std::string &label,
					bool movable)
{
  if(movable)
  {
    ROS_INFO("adding movable");
    // Add a movable object
    Object o = Object(marker);
    o.label = label;
    o.movable = true;
    object_manager_->addObject(o);
  }
  else
  {
    ROS_INFO("adding unmovable");
    // Add a nonmovable object
    Object o = Object(marker);
    o.label = label;
    o.movable = false;
    object_manager_->addObject(o);
  }

  visualizeMesh(marker, attach_interactive_marker);
}

void DemonstrationSceneManager::visualizeMesh(const visualization_msgs::Marker &marker, 
					      bool attach_interactive_marker)
{
  if(attach_interactive_marker)
  {
    if(int_marker_server_ == 0)
    {
      ROS_WARN("[SceneManager%d] Interactive marker server is null", user_id_);
      return;
    }

    // Attach an interactive marker to control this marker.
    visualization_msgs::InteractiveMarker int_marker;
    int_marker.header.frame_id = resolveName("map", user_id_);
    int_marker.pose = marker.pose;

    // Give each interactive marker a unique name according to each mesh's unique id.
    std::stringstream marker_name;
    marker_name << "mesh_marker_" << marker.id;

    int_marker.name = marker_name.str();

    std::stringstream mesh_desc;
    mesh_desc << "Move " << marker_name.str();
    int_marker.description = mesh_desc.str();

    // Add a non-interactive control for the mesh.
    visualization_msgs::InteractiveMarkerControl control;
    control.always_visible = true;
    control.markers.push_back(marker);

    int_marker.controls.push_back(control);

    // Attach a 6-DOF control for moving the mesh around.
    attach6DOFControl(int_marker);

    int_marker_server_->insert(int_marker,
			       mesh_feedback_
			       );
    int_marker_server_->applyChanges();
  }
  else
  {
    ROS_WARN("[DSM] marker frame_id = %s", marker.header.frame_id.c_str());
    marker_pub_.publish(marker);
  }
}

bool DemonstrationSceneManager::updateMeshPose(int mesh_id, const geometry_msgs::Pose &pose)
{
  object_manager_->moveObject(mesh_id, pose);

  return true;
}

void DemonstrationSceneManager::updateMeshScale(int mesh_id, double x, double y, double z)
{
  object_manager_->scaleObject(mesh_id, x, y, z);

  visualizeMesh(object_manager_->getMarker(mesh_id), editMeshesMode());
}

void DemonstrationSceneManager::removeMesh(int mesh_id)
{
  visualization_msgs::Marker mesh = object_manager_->getMarker(mesh_id);

  if(editMeshesMode())
  {
    std::stringstream marker_name;
    marker_name << "mesh_marker_" << mesh_id;

    if(!int_marker_server_->erase(marker_name.str()))
    {
      ROS_ERROR("[SceneManager%d] Failed to remove interactive marker %s!", user_id_, marker_name.str().c_str());
    }
    int_marker_server_->applyChanges();
  }
  else
  {
    mesh.action = visualization_msgs::Marker::DELETE;
    visualizeMesh(mesh, false);
  }

  object_manager_->removeObject(mesh_id);
}

void DemonstrationSceneManager::addGoal(const std::string &desc,
					Goal::GoalType type,
					int object_id,
                                        bool ignore_yaw)
{
  setGoalsChanged(true);

  ROS_INFO("[SceneManager%d] Adding goal type %s with id %d", user_id_, Goal::GoalTypeNames[type], (int)goals_.size());

  switch(type)
  {
  case Goal::PICK_UP:
  {
    PickUpGoal *goal = new PickUpGoal(goals_.size(), desc);

    geometry_msgs::Pose object_pose = object_manager_->getMarker(object_id).pose;
    goal->setGraspPose(object_pose);
    goal->setInitialObjectPose(object_pose);
    goal->setObjectID(object_id);

    goals_.push_back(goal);

    break;
  }
  case Goal::PLACE:
  {
    PlaceGoal *goal = new PlaceGoal(goals_.size(), desc, object_id);

    goal->setIgnoreYaw(ignore_yaw);

    goals_.push_back(goal);

    break;
  }
  default:
    break;
  }

  if(current_goal_ < 0)
    current_goal_ = 0;
}

bool DemonstrationSceneManager::moveGoal(int goal_number, const geometry_msgs::Pose &pose)
{
  if(goal_number < 0 || goal_number >= int(goals_.size()))
  {
    ROS_ERROR("[SceneManager%d] Invalid goal number!", user_id_);
    return false;
  }

  setGoalsChanged(true);

  switch(goals_.at(goal_number)->getType())
  {
  case Goal::PICK_UP:
  { 
    break;
  }
  case Goal::PLACE:
  {
    PlaceGoal *place_goal = static_cast<PlaceGoal *>(goals_.at(goal_number));

    place_goal->setPlacePose(pose);

    drawGoal(place_goal, editGoalsMode());

    break;
  }
  default:
    break;
  }

  return true;
}

Goal *DemonstrationSceneManager::getGoal(int goal_number)
{
  if(goal_number < 0 || goal_number >= int(goals_.size()))
  {
    ROS_ERROR("[SceneManager%d] Invalid goal number!", user_id_);
    return 0;
  }

  return goals_.at(goal_number);
}

std::vector<visualization_msgs::Marker> DemonstrationSceneManager::getMeshes() const
{
  return object_manager_->getMarkers();
}

visualization_msgs::Marker DemonstrationSceneManager::getMeshMarker(int mesh_id) const
{
  return object_manager_->getMarker(mesh_id);
}

std::vector<Object> DemonstrationSceneManager::getObjects() const
{
  return object_manager_->getObjects();
}

std::vector<Goal *> DemonstrationSceneManager::getGoals() const
{
  return goals_;
}

int DemonstrationSceneManager::getNumGoals() const
{
  return goals_.size();
}

bool DemonstrationSceneManager::goalsChanged() const
{
  return goals_changed_;
}

void DemonstrationSceneManager::setGoalsChanged(bool changed)
{
  goals_changed_ = changed;
}

void DemonstrationSceneManager::setGoalDescription(int goal_number, const std::string &desc)
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager%d] Invalid goal number!", user_id_);
    return;
  }

  goals_.at(goal_number)->setDescription(desc);
}

std::string DemonstrationSceneManager::getGoalDescription(int goal_number) const
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager%d] Invalid goal number!", user_id_);
    return "";
  }

  return goals_.at(goal_number)->getDescription();
}

bool DemonstrationSceneManager::setGraspPose(int goal_number, const geometry_msgs::Pose &grasp)
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager%d] Invalid goal number!", user_id_);
    return false;
  }

  if(!goals_.at(goal_number)->getType() == Goal::PICK_UP)
  {
    ROS_ERROR("[SceneManager%d] Can only set the grasp pose for a pick up goal!", user_id_);
    return false;
  }

  PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(getGoal(goal_number));
  pick_up_goal->setGraspPose(grasp);

  return true;
}

geometry_msgs::Pose DemonstrationSceneManager::getGraspPose(int goal_number)
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager%d] Invalid goal number!", user_id_);
    return geometry_msgs::Pose();
  }

  if(!goals_.at(goal_number)->getType() == Goal::PICK_UP)
  {
    ROS_ERROR("[SceneManager%d] Can only get the grasp pose for a pick up goal!", user_id_);
    return geometry_msgs::Pose();
  }

  PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(getGoal(goal_number));
  return pick_up_goal->getGraspPose();  
}

geometry_msgs::Pose DemonstrationSceneManager::getGraspPoseObjectFrame(int goal_number)
{
  if(goal_number < 0 || goal_number >= getNumGoals())
  {
    ROS_ERROR("[SceneManager%d] Invalid goal number!", user_id_);
    return geometry_msgs::Pose();
  }

  if(!goals_.at(goal_number)->getType() == Goal::PICK_UP)
  {
    ROS_ERROR("[SceneManager%d] Can only get the grasp pose for a pick up goal!", user_id_);
    return geometry_msgs::Pose();
  }

  PickUpGoal *g = static_cast<PickUpGoal *>(getGoal(goal_number));
  geometry_msgs::Pose obj_pose = g->getInitialObjectPose();
  geometry_msgs::Pose grasp_pose = g->getGraspPose();
  KDL::Frame obj_in_map(KDL::Rotation::Quaternion(
  			  obj_pose.orientation.x,
  			  obj_pose.orientation.y,
  			  obj_pose.orientation.z,
  			  obj_pose.orientation.w),
  			KDL::Vector(obj_pose.position.x,
  				    obj_pose.position.y,
  				    obj_pose.position.z)
    );
  KDL::Frame grasp_in_map(KDL::Rotation::Quaternion(
  			    grasp_pose.orientation.x,
  			    grasp_pose.orientation.y,
  			    grasp_pose.orientation.z,
  			    grasp_pose.orientation.w),
  			  KDL::Vector(grasp_pose.position.x,
  				      grasp_pose.position.y,
  				      grasp_pose.position.z)
    );
  KDL::Frame grasp_in_obj = obj_in_map.Inverse() * grasp_in_map;
  double x, y, z, w;
  grasp_in_obj.M.GetQuaternion(x, y, z, w);
  ROS_WARN("[DVizUser%d] Pose of grasp in object frame = <(rotation), (translation)> = <(%f, %f, %f, %f), (%f, %f, %f)>", user_id_, x, y, z, w, grasp_in_obj.p.x(), grasp_in_obj.p.y(), grasp_in_obj.p.z());
  
  geometry_msgs::Pose res;
  res.position.x = grasp_in_obj.p.x();
  res.position.y = grasp_in_obj.p.y();
  res.position.z = grasp_in_obj.p.z();
  res.orientation.x = x;
  res.orientation.y = y;
  res.orientation.z = z;
  res.orientation.w = w;

  return res;
}

void DemonstrationSceneManager::processGoalFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  int i;
  for(i = feedback->marker_name.size()-1; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  if(!moveGoal(atoi(feedback->marker_name.substr(i+1).c_str()),
	       feedback->pose))
  {
    ROS_ERROR("[SceneManager%d] Demonstration scene manager failed to update task goal pose!", user_id_);
  }
  else
  {
    setGoalsChanged(true);
  }
}

void DemonstrationSceneManager::processMeshFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback
  )
{
  int i;
  for(i = feedback->marker_name.size()-1; i >= 0; --i)
  {
    if(feedback->marker_name.at(i) == '_')
      break;
  }

  if(!updateMeshPose(atoi(feedback->marker_name.substr(i+1).c_str()),
		     feedback->pose)
     )
  {
    ROS_ERROR("[SceneManager%d] Demonstration scene manager failed to update pose of mesh!", user_id_);
  }  
}

geometry_msgs::Pose DemonstrationSceneManager::getObjectPose(int object_id)
{
  return object_manager_->getMarker(object_id).pose;
}

geometry_msgs::Pose DemonstrationSceneManager::getCurrentGoalPose()
{
  if(getCurrentGoal() < 0 || getCurrentGoal() >= getNumGoals())
  {
    ROS_ERROR("[SceneManager%d] Invalid current goal number!", user_id_);
    return geometry_msgs::Pose();
  }

  switch(goals_.at(getCurrentGoal())->getType())
  {
  case Goal::PICK_UP:
  {
    PickUpGoal *goal = static_cast<PickUpGoal *>(getGoal(getCurrentGoal()));

    return goal->getGraspPose();

    break;
  }
  case Goal::PLACE:
  {
    PlaceGoal *goal = static_cast<PlaceGoal *>(getGoal(getCurrentGoal()));

    return goal->getPlacePose();

    break;
  }
  default:
  {
    ROS_ERROR("[SceneManager%d] Unknown goal type!", user_id_);
    return geometry_msgs::Pose();
  }
  }
}

int DemonstrationSceneManager::getCurrentGoal() const
{
  return current_goal_;
}

void DemonstrationSceneManager::setCurrentGoal(int goal)
{
  if(goal != current_goal_)
  {
    current_goal_ = goal;
    setGoalsChanged(true);
  }
}

bool DemonstrationSceneManager::editGoalsMode() const
{
  return edit_goals_mode_;
}

void DemonstrationSceneManager::setEditGoalsMode(bool edit)
{
  edit_goals_mode_ = edit;
}

bool DemonstrationSceneManager::editMeshesMode() const
{
  return edit_meshes_mode_;
}

void DemonstrationSceneManager::setEditMeshesMode(bool edit)
{
  if(edit != edit_meshes_mode_)
  {
    std::vector<visualization_msgs::Marker> meshes = object_manager_->getMarkers();
    if(edit)
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	it->header.frame_id = resolveName("map", user_id_);
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::DELETE;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->mesh_use_embedded_materials = true;

	// First remove the old markers.
	marker_pub_.publish(*it);

	// Then add the markers again, but this time with interactive markers.
	it->action = visualization_msgs::Marker::ADD;
	visualizeMesh(*it, true);
      }
    }
    else
    {
      std::vector<visualization_msgs::Marker>::iterator it;
      // For each mesh, first remove all the interactive markers from the meshes.
      // Then, re-visualize each marker without an attached interactive marker.
      for(it = meshes.begin(); it != meshes.end(); ++it)
      {
	std::stringstream int_marker_name;
	int_marker_name << "mesh_marker_" << it->id;

	if(!int_marker_server_->erase(int_marker_name.str()))
	{
	  ROS_ERROR("[SceneManager%d] Failed to remove interactive marker on mesh %d!", user_id_, it->id);
	}
	int_marker_server_->applyChanges();

	it->header.frame_id = resolveName("map", user_id_);
	it->header.stamp = ros::Time();
	it->action = visualization_msgs::Marker::ADD;
	it->type = visualization_msgs::Marker::MESH_RESOURCE;
	it->color.r = it->color.g = it->color.b = it->color.a = 0;
	it->mesh_use_embedded_materials = true;

	marker_pub_.publish(*it);
      }
    }

    edit_meshes_mode_ = edit;
  }
}

bool DemonstrationSceneManager::taskDone() const
{
  return (current_goal_ >= (int)goals_.size());
}

geometry_msgs::Pose DemonstrationSceneManager::getInitialRobotPose() const
{
  return initial_robot_pose_;
}

double DemonstrationSceneManager::getInitialTorsoPosition() const
{
  return initial_torso_position_;
}

void DemonstrationSceneManager::drawGoal(Goal *goal, bool attach_interactive_marker)
{
  switch(goal->getType())
  {
  case Goal::PICK_UP:
  {
    PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(goal);

    // Draw the shadow of the gripper around the object, where the user
    // has selected the grasp.
    if(pick_up_goal->isGraspDone())
    {
      geometry_msgs::Pose grasp_pose = pick_up_goal->getGraspPose();

      // Combine the grasp marker pose in the map frame with the 
      // grasp distance to get the actual grasp pose in the map
      // frame.
      tf::Transform marker_in_map(tf::Quaternion(grasp_pose.orientation.x,
						 grasp_pose.orientation.y,
						 grasp_pose.orientation.z,
						 grasp_pose.orientation.w),
				  tf::Vector3(grasp_pose.position.x,
					      grasp_pose.position.y,
					      grasp_pose.position.z)
	);

      tf::Transform gripper_in_marker(tf::Quaternion::getIdentity(),
				      tf::Vector3(/*-1.0*pick_up_goal->getGraspDistance()*/0.0,
						  0.0,
						  0.0)
	);

      tf::Transform gripper_in_map = marker_in_map * gripper_in_marker;

      geometry_msgs::Pose grasp_pose_fixed;
      tf::quaternionTFToMsg(gripper_in_map.getRotation(), grasp_pose_fixed.orientation);
      geometry_msgs::Vector3 position;
      tf::vector3TFToMsg(gripper_in_map.getOrigin(), position);
      grasp_pose_fixed.position.x = position.x;
      grasp_pose_fixed.position.y = position.y;
      grasp_pose_fixed.position.z = position.z;

      std::vector<visualization_msgs::Marker> gripper_markers;
      // pviz_->getGripperMeshesMarkerMsg(grasp_pose_fixed, 0.2, GOAL_MARKER_NAMESPACE, 
      // 				 5*pick_up_goal->getGoalNumber(), true, gripper_markers);
      pviz_->getGripperMeshesMarkerMsg(grasp_pose_fixed, 0.2, GOAL_MARKER_NAMESPACE, 
				       5*pick_up_goal->getGoalNumber(), 
				       pick_up_goal->getGripperJointPosition(), gripper_markers);

      ROS_DEBUG("[SceneManager%d] PICK UP GOAL", user_id_);

      for(int i = 0; i < int(gripper_markers.size()); ++i)
      {
	gripper_markers[i].header.frame_id = resolveName("map", user_id_);
	// @todo for the web version, we must change the path of the gripper meshes.
	// Eliminate the "package://pr2_description/" prefix and replace with 
	// "http://resources.robotwebtools.org/" to make the meshes web-compatible.
	// gripper_markers[i].mesh_resource = "http://resources.robotwebtools.org/"
	//   + gripper_markers[i].mesh_resource.substr(10);
	ROS_DEBUG("[SceneManager%d] Gripper marker path = %s", user_id_, gripper_markers[i].mesh_resource.c_str());
	gripper_markers[i].color.a = 0.3;
	marker_pub_.publish(gripper_markers[i]);
      }
    }

    break;
  }
  case Goal::PLACE:
  {
    PlaceGoal *place_goal = static_cast<PlaceGoal *>(goal);

    // Draw a shadow of the object that can be moved around to indicate
    // where the user should place the object. 
    visualization_msgs::Marker object = object_manager_->getMarker(place_goal->getObjectID());
      
    object.header.frame_id = resolveName("map", user_id_);
    object.header.stamp = ros::Time();
    object.ns = "dviz_place_goal";
    object.id = place_goal->getGoalNumber();
    object.type = visualization_msgs::Marker::MESH_RESOURCE;
    object.action = visualization_msgs::Marker::ADD;
    object.pose = place_goal->getPlacePose();
    object.mesh_use_embedded_materials = false;
    object.color.r = 0;
    object.color.g = 0;
    object.color.b = 1;
    object.color.a = 0.4;

    if(attach_interactive_marker)
    {
      // Attach an interactive marker to control this marker.
      visualization_msgs::InteractiveMarker int_marker;
      int_marker.header.frame_id = resolveName("map", user_id_);
      int_marker.pose = object.pose;

      // Give each interactive marker a unique name according to each goal's unique id.
      std::stringstream marker_name;
      marker_name << "goal_marker_" << object.id;

      int_marker.name = marker_name.str();

      std::stringstream mesh_desc;
      mesh_desc << "Move " << marker_name.str();
      int_marker.description = mesh_desc.str();

      // Add a non-interactive control for the mesh.
      visualization_msgs::InteractiveMarkerControl control;
      control.always_visible = true;
      control.markers.push_back(object);

      int_marker.controls.push_back(control);

      // Attach a 6-DOF control for moving the place goal around.
      attach6DOFControl(int_marker);

      int_marker_server_->insert(int_marker, goal_feedback_);
      int_marker_server_->applyChanges();
    }
    else
    {
      marker_pub_.publish(object);
    }

    break;
  }
  default:
    break;
  }
}

void DemonstrationSceneManager::hideGoal(Goal *goal)
{
  switch(goal->getType())
  {
  case Goal::PICK_UP:
  {
    PickUpGoal *pick_up_goal = static_cast<PickUpGoal *>(goal);

    std::vector<visualization_msgs::Marker> gripper_markers;
    pviz_->getGripperMeshesMarkerMsg(pick_up_goal->getGraspPose(), 0.2, 
				     GOAL_MARKER_NAMESPACE, 5*pick_up_goal->getGoalNumber(), 
				     pick_up_goal->getGripperJointPosition(), gripper_markers);

    for(int i = 0; i < int(gripper_markers.size()); ++i)
    {
      gripper_markers[i].header.frame_id = resolveName("map", user_id_);
      // @todo for the web version, we must change the path of the gripper meshes.
      // Eliminate the "package://pr2_description/" prefix and replace with 
      // "http://resources.robotwebtools.org/" to make the meshes web-compatible.
      // gripper_markers[i].mesh_resource = "http://resources.robotwebtools.org/"
      // 	+ gripper_markers[i].mesh_resource.substr(10);

      gripper_markers[i].action = visualization_msgs::Marker::DELETE;
      marker_pub_.publish(gripper_markers[i]);
    }

    break;
  }
  case Goal::PLACE:
  {
    PlaceGoal *place_goal = static_cast<PlaceGoal *>(goal);

    visualization_msgs::Marker object = object_manager_->getMarker(place_goal->getObjectID());
    int goal_number = place_goal->getGoalNumber();

    std::stringstream marker_name;
    marker_name << "goal_marker_" << goal_number;

    int_marker_server_->erase(marker_name.str());
    int_marker_server_->applyChanges();

    object.header.frame_id = resolveName("map", user_id_);
    object.header.stamp = ros::Time();
    object.ns = "dviz_place_goal";
    object.id = place_goal->getGoalNumber();
    object.action = visualization_msgs::Marker::DELETE;

    marker_pub_.publish(object);

    break;
  }
  default:
    break;
  }
}

std::string DemonstrationSceneManager::getTaskName() const
{
  return task_name_;
}

void DemonstrationSceneManager::setTaskName(const std::string &name)
{
  task_name_ = name;
}

} // namespace demonstration_visualizer
