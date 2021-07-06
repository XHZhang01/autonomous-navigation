#include "ros/ros.h"
#include <cstdlib>
#include <std_srvs/Empty.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "octomap_reset_client");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/octomap_server/reset");
  std_srvs::Empty empty;


   

   if (argc == 2)
   {
     ROS_INFO("reset after %f s",atof(argv[1]));
     ros::Duration(atof(argv[1])).sleep();
     ROS_INFO("reset now!");

			if (client.call(empty))
			{
				ROS_INFO("Successfully reset the map");
			}
			else
			{
				ROS_ERROR("Failed to call service octomap_server/reset");
				return 1;
			}		
     return 0;
   }

  

//  client.call(empty);
  if (client.call(empty))
  {
    ROS_INFO("Successfully reset the map");
  }
  else
  {
    ROS_ERROR("Failed to call service octomap_server/reset");
    return 1;
  }

  return 0;
}