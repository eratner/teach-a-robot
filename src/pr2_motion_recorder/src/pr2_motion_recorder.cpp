#include "pr2_motion_recorder.h"

PR2MotionRecorder::PR2MotionRecorder()
  : is_recording_(false),
    is_replaying_(false),
    bag_count_(0), 
    write_bag_path_(""),
    write_bag_(),
    read_bag_()
{
  ros::NodeHandle nh("~");

  begin_rec_service_ = nh.advertiseService("begin_recording", 
					   &PR2MotionRecorder::beginRecording,
					   this);

  end_rec_service_ = nh.advertiseService("end_recording",
					 &PR2MotionRecorder::endRecording,
					 this);

  begin_replay_service_ = nh.advertiseService("begin_replay",
					      &PR2MotionRecorder::beginReplay,
					      this);

  end_replay_service_ = nh.advertiseService("end_replay",
					    &PR2MotionRecorder::endReplay,
					    this);

  joint_states_subscription_ = nh.subscribe("/joint_states", 
					    100,
					    &PR2MotionRecorder::recordJoints,
					    this);

  // Listen to the ground truth pose of the base.
  base_pose_subscription_ = nh.subscribe("/base_pose_ground_truth",
					 100,
					 &PR2MotionRecorder::recordBasePose,
					 this);

  nh.param("write_bag_path", write_bag_path_, std::string(""));

  // For replaying the arm motions, use joint trajectory actions.
  r_arm_traj_client_ = new TrajectoryClient("r_arm_controller/joint_trajectory_action", true);

  while(!r_arm_traj_client_->waitForServer(ros::Duration(5.0)))
  {
    ROS_INFO("Waiting for joint_trajectory_action server (r_arm_controller).");
  }
}

PR2MotionRecorder::~PR2MotionRecorder()
{
  delete r_arm_traj_client_;
  r_arm_traj_client_ = 0;
}

bool PR2MotionRecorder::beginRecording(pr2_motion_recorder::FilePath::Request  &req,
				       pr2_motion_recorder::FilePath::Response &res)
{
  if(!is_recording_)
  {
    is_recording_ = true;
    // Start recording to a new bag file.
    std::stringstream path;
    path << req.file_path << "/motion" << bag_count_ << ".bag";
    bag_count_++;
    write_bag_.open(path.str(), rosbag::bagmode::Write);
    ROS_INFO("Beginning to record motion to %s.", path.str().c_str());
  }

  return true;
}

bool PR2MotionRecorder::endRecording(std_srvs::Empty::Request  &req,
				     std_srvs::Empty::Response &res)
{
  if(is_recording_)
  {
    is_recording_ = false;
    // Stop recording.
    ROS_INFO("Recording finished with %d joint states.", write_bag_.getSize());
    write_bag_.close();
  }

  return true;
}

bool PR2MotionRecorder::beginReplay(pr2_motion_recorder::FilePath::Request  &req,
				    pr2_motion_recorder::FilePath::Response &res)
{
  // Load appropriate bag file, populate a JointTrajectoryGoal for each controller
  // and pass the to appropriate trajectory action client. 
  read_bag_.open(req.file_path, rosbag::bagmode::Read);

  rosbag::View joints_view(read_bag_, rosbag::TopicQuery("/joint_states"));

  // RIGHT ARM JOINTS.
  pr2_controllers_msgs::JointTrajectoryGoal r_arm_goal;

  // Set the joint names.
  sensor_msgs::JointState::ConstPtr first_joint_state = 
    joints_view.begin()->instantiate<sensor_msgs::JointState>();
  for(int i = 17; i < 24; ++i)
  {
    ROS_INFO("Joint %d: %s", i, first_joint_state->name[i].c_str());
    r_arm_goal.trajectory.joint_names.push_back(first_joint_state->name[i]);
  }

  // Get the number of messages recorded.
  ROS_INFO("Constructing a joint trajectory with %d waypoints.", joints_view.size());
  r_arm_goal.trajectory.points.resize(joints_view.size());

  int index = 0;
  foreach(rosbag::MessageInstance const m, joints_view)
  {
    sensor_msgs::JointState::ConstPtr joint_state = m.instantiate<sensor_msgs::JointState>();
    if(joint_state != NULL)
    {
      // Insert the position and velocities of the joints as a waypoint.
      r_arm_goal.trajectory.points[index].positions.resize(7);
      r_arm_goal.trajectory.points[index].velocities.resize(7);
      for(int i = 0; i < 7; ++i)
      {
	r_arm_goal.trajectory.points[index].positions[i] = joint_state->position[17+i];
	r_arm_goal.trajectory.points[index].velocities[i] = 0; /*joint_state->velocity[17+i];*/
      }
      // To be reached index*0.1 seconds after beginning the trajectory.
      r_arm_goal.trajectory.points[index].time_from_start = ros::Duration(index*0.1);  
      index++;
    }
  }
  ROS_INFO("Added %d waypoints.", index);

  read_bag_.close();

  if(!is_replaying_)
    is_replaying_ = true;

  startJointTrajectory(r_arm_goal, r_arm_traj_client_);

  return true;
}

bool PR2MotionRecorder::endReplay(std_srvs::Empty::Request  &,
				  std_srvs::Empty::Response &)
{
  // @todo
  return true;
}

void PR2MotionRecorder::recordJoints(const sensor_msgs::JointState &msg)
{
  if(is_recording_)
  {
    write_bag_.write("/joint_states", ros::Time::now(), msg);
  }
}

void PR2MotionRecorder::recordBasePose(const nav_msgs::Odometry &msg)
{
  if(is_recording_)
  {
    write_bag_.write("/base_pose_ground_truth", ros::Time::now(), msg);
  }
}

void PR2MotionRecorder::run()
{
  ros::Rate loop_rate(10.0);

  while(ros::ok())
  {
    // Check if replaying is done.
    if(is_replaying_ && r_arm_traj_client_->getState().isDone())
    {
      ROS_INFO("Replay is ending.");
      is_replaying_ = false;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void PR2MotionRecorder::startJointTrajectory(pr2_controllers_msgs::JointTrajectoryGoal goal,
					     TrajectoryClient *trajectory_client)
{
  // Start in 1 s from now.
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);
  trajectory_client->sendGoal(goal);
}
