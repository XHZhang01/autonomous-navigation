
#include <ros/ros.h>
#include "rosgraph_msgs/Clock.h"

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "time_server");
  ros::NodeHandle n;
  ros::Publisher time_server;
  time_server     = n.advertise<rosgraph_msgs::Clock>("/clock", 1);
  rosgraph_msgs::Clock clock;
  ros::WallTime start(ros::WallTime::now());
  ros::Rate loop_rate(1000);

  while (ros::ok()) {    
    double t = (ros::WallTime::now()-start).toSec();
    clock.clock = ros::Time(t);

    time_server.publish(clock);
    ros::spinOnce();
  }

  return 0;
}
