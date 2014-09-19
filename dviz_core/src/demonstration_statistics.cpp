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

std::vector<double> getTotalJointDistances(const dviz_core::Step &step, bool ouput = false);

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
  // First 3 DoFs are (x, y, yaw) of the base; last 7 DoFs are the joints of the arm
  std::vector<std::vector<double> > total_joint_distance(12, std::vector<double>(10, 0.0f));

  rosbag::View view(bag, rosbag::TopicQuery("/demonstration"));
  if(output) ROS_INFO("View contains %d saves", view.size());
  rosbag::View::iterator iter = view.begin();
  bool printed_joint_names = false;
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
        for(unsigned i = 0; i < 6; ++i)
        {
          if(GoalName[i].compare(loaded_demo->steps[i].object_type) == 0)
          {
            int index = pick_up ? 2*i : 2*i + 1;
            total_joint_distance[index] = getTotalJointDistances(loaded_demo->steps[i], output && !printed_joint_names);
            if (output && !printed_joint_names) printed_joint_names = true;
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
    std::cout << GoalName[i / 2] << " " << (i % 2 == 0 ? PickUp : Place);
    for (unsigned j = 0; j < 10; ++j)
    {
      std::cout << " " << total_joint_distance[i][j];
      total += total_joint_distance[i][j];
    }
    std::cout << " " << total << std::endl;
  }

  return 0;
}

std::vector<double> getTotalJointDistances(const dviz_core::Step &step, bool ouput)
{
  std::vector<double> distances(10, 0.0f);
  for(unsigned i = 0; i < step.waypoints.size() - 1; ++i)
  {
    distances[0] += (std::abs(step.waypoints[i + 1].base_pose.position.x - step.waypoints[i].base_pose.position.x));
    distances[1] += (std::abs(step.waypoints[i + 1].base_pose.position.x - step.waypoints[i].base_pose.position.x));
    distances[2] += (angles::shortest_angular_distance(tf::getYaw(step.waypoints[i].base_pose.orientation), tf::getYaw(step.waypoints[i + 1].base_pose.orientation)));
    for(unsigned j = 0; j < 7; ++j)
    {
      std::cout << j << " " << step.waypoints[i].joint_states.name[j] << std::endl;
      distances[2 + j] += (angles::shortest_angular_distance(step.waypoints[i].joint_states.position[j], step.waypoints[i + 1].joint_states.position[j]));
    }
  }
  return distances;
}
