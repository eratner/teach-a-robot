#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <dviz_core/UserDemonstration.h>
#include <dviz_core/Step.h>
#include <dviz_core/Waypoint.h>
#include <angles/angles.h>
#include <tf/transform_datatypes.h>

// Kitchen scene-specific constants
enum GoalNumber
{
  OIL_PICK,
  OIL_PLACE,
  CAKEMIX_PICK,
  CAKEMIX_PLACE,
  WATER_PICK,
  WATER_PLACE,
  PAN_PICK,
  PAN_PLACE,
  HOCKEYSTICK_PICK,
  HOCKEYSTICK_PLACE,
  EGGS_PICK,
  EGGS_PLACE,
  EOT
};

const std::string GoalName[] =
{
  "oil",
  "cake_mix",
  "water_bottle",
  "cake_pan",
  "hockey_stick",
  "eggs",
  "EOT"
};

const std::string PickUp = "Pick Up";
const std::string Place = "Place";

geometry_msgs::Pose makePose(double p_x, double p_y, double p_z, double o_x = 0.0f, double o_y = 0.0f, double o_z = 0.0f, double o_w = 1.0f);

// These are taken from scenes/kitchen_lite.xml
const geometry_msgs::Pose InitialObjectPoses[] =
{
  makePose(-1.51785, 1.18239, 1.01662),     // Oil
  makePose(-1.53422, 0.0206066, 1.47446),   // Cake mix
  makePose(1.51495, 4.11129, 0.903346),     // Water bottle
  makePose(0.480567, 1.58575, 0.702783),    // Cake pan
  makePose(4.93094, 4.00069, 0.510713),     // Hockey stick
  makePose(0.635449, 4.29259,0.940121)      // Eggs
};

double euclideanDistance(const geometry_msgs::Pose &first, const geometry_msgs::Pose &second);

std::vector<double> getTotalJointDistances(const dviz_core::Step &step, bool output = false);

int main(int argv, char **argc)
{
  if(argv < 2)
  {
    ROS_WARN("Usage: %s demonstration_bagfile", argc[0]);
    return 1;
  }

  bool output = true;
  if(argv > 2)
  {
    if(strcmp(argc[2], "false") == 0)
    {
      output = false;
    }
  }

  rosbag::Bag bag;
  std::string file(argc[1]);

  if(output) ROS_INFO("Reading demonstration bagfile %s", argc[1]);

  try
  {
    bag.open(file, rosbag::bagmode::Read);
  }
  catch(const rosbag::BagException &e)
  {
    ROS_ERROR("Failed to open bagfile %s: %s", file.c_str(), e.what());
    return 1;
  }

  // Note that total_joint_distance[i][j] = for the ith goal, the total angular distance travelled by the jth DoF
  // First 3 DoFs are (x, y, yaw) of the base; last 7 DoFs are the joints of the arm:
  //  3 l_shoulder_pan_joint
  //  4 l_shoulder_lift_joint
  //  5 l_upper_arm_roll_joint
  //  6 l_elbow_flex_joint
  //  7 l_forearm_roll_joint
  //  8 l_wrist_flex_joint
  //  9 l_wrist_roll_joint

  std::vector<std::vector<double> > total_joint_distance(12, std::vector<double>(10, 0.0f));

  rosbag::View view(bag, rosbag::TopicQuery("/demonstration"));
  if(output) ROS_INFO("View contains %d saves", view.size());
  rosbag::View::iterator iter = view.begin();
  dviz_core::UserDemonstration::ConstPtr loaded_demo;
  for(; iter != view.end(); iter++)
  {
    dviz_core::UserDemonstration::ConstPtr loaded_demo = (*iter).instantiate<dviz_core::UserDemonstration>();
    if(loaded_demo == NULL)
    {
      ROS_ERROR("Demonstration failed to load!");
      return false;
    }
    // Get information about the user demonstration
    if(output)
    {
      ROS_INFO("Demonstration from user %s with id %s performing task \"%s\" on the date %s with duration %f second and the following steps:",
               loaded_demo->user_id.c_str(),
               loaded_demo->demo_id.c_str(),
               loaded_demo->task_name.c_str(),
               loaded_demo->date.c_str(),
               loaded_demo->demo_duration.toSec());
    }
    for(int i = 0; i < loaded_demo->steps.size(); i++)
    {
      if(output)
      {
        ROS_INFO("\t [Step %d] goal number: %d; action: \"%s\"; object type: \"%s\"; waypoints: %d; duration %f",
                 i,
                 loaded_demo->steps[i].goal_number,
                 loaded_demo->steps[i].action.c_str(),
                 loaded_demo->steps[i].object_type.c_str(),
                 (int)loaded_demo->steps[i].waypoints.size(),
                 loaded_demo->steps[i].step_duration.toSec());
      }
      if(loaded_demo->steps[i].step_duration.toSec() > 0)
      {
        // Determine whether it is a pick up or place
        bool pick_up;
        if(loaded_demo->steps[i].action.compare(PickUp) == 0)
        {
          // Pick Up
          pick_up = true;
        }
        else if(loaded_demo->steps[i].action.compare(Place) == 0)
        {
          // Place
          pick_up = false;
        }
        else
        {
          // Invalid action
          break;
        }
        for(unsigned j = 0; j < 6; ++j)
        {
          if(GoalName[j].compare(loaded_demo->steps[i].object_type) == 0)
          {
            int index = pick_up ? 2*j : 2*j + 1;
            total_joint_distance[index] = getTotalJointDistances(loaded_demo->steps[i], output);
            break;
          }
        }
      }
    }
  }
  bag.close();

  // Output statistics about the bagfile
  for(unsigned i = 0; i < 12; ++i)
  {
    double total = 0.0f;
    std::cout << GoalName[i / 2] << " " << (i % 2 == 0 ? "PickUp" : "Place");
    for (unsigned j = 0; j < 10; ++j)
    {
      std::cout << " " << total_joint_distance[i][j];
      if (j > 2)
        total += total_joint_distance[i][j];
    }
    std::cout << " " << total << std::endl;
  }

  return 0;
}

geometry_msgs::Pose makePose(double p_x, double p_y, double p_z, double o_x, double o_y, double o_z, double o_w)
{
  geometry_msgs::Pose pose;
  pose.position.x = p_x;
  pose.position.y = p_y;
  pose.position.z = p_z;
  pose.orientation.x = o_x;
  pose.orientation.y = o_y;
  pose.orientation.z = o_z;
  pose.orientation.w = o_w;
  return pose;
}

double euclideanDistance(const geometry_msgs::Pose &first, const geometry_msgs::Pose &second)
{
  return std::sqrt(std::pow(first.position.x - second.position.x, 2) +
                   std::pow(first.position.y - second.position.y, 2) +
                   std::pow(first.position.z - second.position.z, 2));
}

std::vector<double> getTotalJointDistances(const dviz_core::Step &step, bool output)
{
  std::vector<double> distances(10, 0.0f);
  for(unsigned i = 0; i < step.waypoints.size() - 1; ++i)
  {
    distances[0] += (std::abs(step.waypoints[i + 1].base_pose.position.x - step.waypoints[i].base_pose.position.x));
    distances[1] += (std::abs(step.waypoints[i + 1].base_pose.position.y - step.waypoints[i].base_pose.position.y));
    distances[2] += (std::abs(tf::getYaw(step.waypoints[i + 1].base_pose.orientation) - tf::getYaw(step.waypoints[i].base_pose.orientation)));
    for(unsigned j = 0; j < 7; ++j)
    {
//      if (output) std::cout << step.waypoints[i].joint_states.name[j] << " diff = " << step.waypoints[i + 1].joint_states.position[j] << " - " << step.waypoints[i].joint_states.position[j] << std::endl;
      distances[3 + j] += (std::abs(step.waypoints[i + 1].joint_states.position[j + 7] - step.waypoints[i].joint_states.position[j + 7]));
    }
//    if (output) std::cout << std::endl;
  }
  return distances;
}
