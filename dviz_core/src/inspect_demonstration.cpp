#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <dviz_core/UserDemonstration.h>
#include <dviz_core/Step.h>
//#include <dviz_core/OldUserDemonstration.h>
//#include <dviz_core/OldStep.h>
#include <dviz_core/Waypoint.h>

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

void recordGoalCompleted(std::vector<bool> &completed,
			 const std::string &action,
			 const std::string &object,
			 std::vector<double> &times,
                         double time = 0.0f);

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

  int total_goals_completed = 0;
  double total_time = 0.0f;
  std::string user = "";
  std::vector<bool> goal_completed(12, false);
  std::vector<double> goal_time(12, 0.0f);

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

  rosbag::View view(bag, rosbag::TopicQuery("/demonstration"));
  if(output) ROS_INFO("View contains %d saves", view.size());
  rosbag::View::iterator iter = view.begin();
  dviz_core::UserDemonstration::ConstPtr new_loaded_demo;
//  dviz_core::OldUserDemonstration::ConstPtr old_loaded_demo;
  for(; iter != view.end(); iter++)
  {
    dviz_core::UserDemonstration::ConstPtr new_loaded_demo = (*iter).instantiate<dviz_core::UserDemonstration>();
    if(new_loaded_demo == NULL)
    {
      // Try the old bagfile type
//      if(output) ROS_ERROR("Failed to load (new) user demonstration from bag file!");
//      old_loaded_demo = (*iter).instantiate<dviz_core::OldUserDemonstration>();
//      if(old_loaded_demo == NULL)
//      {
//	if(output) ROS_ERROR("Failed to load (old) user demonstration from bag file");
//	return false;
//      }
      ROS_ERROR("New demonstration failed to load, and old demonstration format not supported!");
      return false;
    }

    int goals_completed = 0;
    double time = 0.0f;
    if(new_loaded_demo)
    {
      // Get information about the (new) user demonstration
      if(output)
      {
	ROS_INFO("Demonstration from user %s with id %s performing task \"%s\" on the date %s with duration %f second and the following steps:",
		 new_loaded_demo->user_id.c_str(),
		 new_loaded_demo->demo_id.c_str(),
		 new_loaded_demo->task_name.c_str(),
		 new_loaded_demo->date.c_str(),
		 new_loaded_demo->demo_duration.toSec());
      }
      user = new_loaded_demo->user_id;
      for(int i = 0; i < new_loaded_demo->steps.size(); i++)
      {
	if(output)
	{
	  ROS_INFO("\t [Step %d] goal number: %d; action: \"%s\"; object type: \"%s\"; waypoints: %d; duration %f",
		   i,
		   new_loaded_demo->steps[i].goal_number,
		   new_loaded_demo->steps[i].action.c_str(),
		   new_loaded_demo->steps[i].object_type.c_str(),
		   (int)new_loaded_demo->steps[i].waypoints.size(),
		   new_loaded_demo->steps[i].step_duration.toSec());
	}
	if(new_loaded_demo->steps[i].step_duration.toSec() > 0)
	{
	  goals_completed += 1;
	  time += new_loaded_demo->steps[i].step_duration.toSec();
	  recordGoalCompleted(goal_completed, new_loaded_demo->steps[i].action, new_loaded_demo->steps[i].object_type, goal_time, new_loaded_demo->steps[i].step_duration.toSec());
	}
      }
    }
//    else
//    {
//      // Get information about the user demonstration
//      if(output)
//      {
//	ROS_INFO("Demonstration from user %s with id %s performing task \"%s\" on the date %s with the following steps:",
//		 old_loaded_demo->user_id.c_str(),
//		 old_loaded_demo->demo_id.c_str(),
//		 old_loaded_demo->task_name.c_str(),
//		 old_loaded_demo->date.c_str());
//      }
//      user = old_loaded_demo->user_id;
//      for(int i = 0; i < old_loaded_demo->steps.size(); i++)
//      {
//	if(output)
//	{
//	  ROS_INFO("\t [Step %d] goal number: %d; action: \"%s\"; object type: \"%s\"; waypoints: %d",
//		   i,
//		   old_loaded_demo->steps[i].goal_number,
//		   old_loaded_demo->steps[i].action.c_str(),
//		   old_loaded_demo->steps[i].object_type.c_str(),
//		   (int)old_loaded_demo->steps[i].waypoints.size());
//	}
//	if(i < old_loaded_demo->steps.size()-1)
//	{
//	  goals_completed += 1;
//	  recordGoalCompleted(goal_completed, old_loaded_demo->steps[i].action, old_loaded_demo->steps[i].object_type, goal_time);
//	}
//      }
//    }
//    total_goals_completed = std::max(total_goals_completed, goals_completed);
//    total_time = std::max(total_time, time);
  }
  bag.close();

  // Output statistics about the bagfile
  std::cout << user << " " << total_goals_completed << " " << total_time << " ";
  for(int i = 0; i < goal_completed.size(); ++i)
  {
    std::cout << (goal_completed[i] ? "1 " : "0 ");
  }
  for(int i = 0; i < goal_time.size(); ++i)
  {
    std::cout << goal_time[i] << " ";
  }
  std::cout << std::endl;

  return 0;
}

void recordGoalCompleted(std::vector<bool> &completed, 
			 const std::string &action,
			 const std::string &object,
                         std::vector<double> &times,
                         double time)
{
  bool pick_up;
  if(action.compare(PickUp) == 0)
  {
    // Pick Up 
    pick_up = true;
  }
  else if(action.compare(Place) == 0)
  {
    // Place
    pick_up = false;
  }
  else
  {
    // Invalid action
    return;
  }

  for(int i = 0; i < 6; ++i)
  {
    if(GoalName[i].compare(object) == 0)
    {
      int index = pick_up ? 2*i : 2*i + 1;
      completed[index] = true;

      if(time > 0)
	times[index] = time;

      break;
    }
  }
}
