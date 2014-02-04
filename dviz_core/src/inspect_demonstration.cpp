#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <dviz_core/UserDemonstration.h>
#include <dviz_core/Step.h>
#include <dviz_core/Waypoint.h>

int main(int argv, char **argc)
{
  if(argv < 2)
  {
    ROS_WARN("Usage: %s demonstration_bagfile", argc[0]);
    return 1;
  }

  ROS_INFO("Reading demonstration bagfile %s", argc[1]);

  rosbag::Bag bag;
  std::string file(argc[1]);

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
  ROS_INFO("View contains %d saves", view.size());
  rosbag::View::iterator iter = view.begin();
  for(; iter != view.end(); iter++)
  {
    dviz_core::UserDemonstration::ConstPtr loaded_demo = (*iter).instantiate<dviz_core::UserDemonstration>();
    if(loaded_demo == NULL)
    {
      ROS_ERROR("Failed to load user demonstration from bag file!");
      return false;
    }

    // Get information about the user demonstration.
    ROS_INFO("Demonstration from user %s with id %s performing task \"%s\" on the date %s with the following steps:",
	     loaded_demo->user_id.c_str(),
	     loaded_demo->demo_id.c_str(),
	     loaded_demo->task_name.c_str(),
	     loaded_demo->date.c_str());
    for(int i = 0; i < loaded_demo->steps.size(); i++)
    {
      ROS_INFO("\t [Step %d] goal number: %d; action: \"%s\"; object type: \"%s\"; waypoints: %d",
	       i,
	       loaded_demo->steps[i].goal_number,
	       loaded_demo->steps[i].action.c_str(),
	       loaded_demo->steps[i].object_type.c_str(),
	       (int)loaded_demo->steps[i].waypoints.size());
    }
  }
  bag.close();

  return 0;
}
