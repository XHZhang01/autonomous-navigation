#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <cstdlib>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include <sstream>
 
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_test");
  ros::NodeHandle n;
  ros::Publisher goal_sent_pub = n.advertise<std_msgs::Bool>("goal_sent", 1);
  ros::Publisher goal_status_pub = n.advertise<std_msgs::Bool>("goal_status", 1);

  double x = 100;
  double y = 0;
  double z = 0;
 
  //订阅move_base服务器的消息  
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base("move_base", true);
 
  //等待连接服务器，5s等待时间限制 
  while(!move_base.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for move_base action server...");
  }
 
  ROS_INFO("Connected to move base server");

   if (argc == 4)
   {
     ROS_INFO("set goal to x %f y %f z %f",atof(argv[1]),atof(argv[2]),atof(argv[3]));

      x = atof(argv[1]);
      y = atof(argv[2]);
      z = atof(argv[3]);
   }



  //设定目标点
  move_base_msgs::MoveBaseGoal goal;
  goal.target_pose.header.frame_id = "world";
  goal.target_pose.header.stamp = ros::Time::now();
 
  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.position.z = z;

  goal.target_pose.pose.orientation.w = 1;
 
  ROS_INFO("Sending goal");
  move_base.sendGoal(goal);
  std_msgs::Bool flag;
  flag.data = true;
  goal_sent_pub.publish(flag);

  move_base.waitForResult();
 
  if(move_base.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {  
    ROS_INFO("Goal succeeded!");
    std_msgs::Bool goalstate;
    goalstate.data = true;
    goal_status_pub.publish(goalstate);



  }else
  {
    ROS_INFO("Goal failed");
  }
  return 0;

}
