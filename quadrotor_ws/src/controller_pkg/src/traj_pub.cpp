#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>
#include <sstream>
#include <iostream>

#include <math.h>

#define STATIC_POSE 1

#define PI M_PI

#define TFOUTPUT 1

geometry_msgs::Twist vel_got;
bool goal_reached = false;
ros::Time time_goal_reached;


void cmd_vel_get(const geometry_msgs::Twist& vel)
{
        vel_got = vel;

}
void goal_status_get(const std_msgs::Bool& goal_status)
{
        goal_reached = goal_status.data;
        time_goal_reached = ros::Time::now();
        
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "traj_publisher");
    ros::NodeHandle n;
    ros::Publisher desired_state_pub = n.advertise<trajectory_msgs::MultiDOFJointTrajectoryPoint>("desired_state", 1);

    ros::Publisher take_off_state_pub = n.advertise<std_msgs::String>("TakeOff", 1);
    ros::Publisher traveling_state_pub = n.advertise<std_msgs::String>("Traveling", 1);
    ros::Publisher landing_state_pub = n.advertise<std_msgs::String>("Landing", 1);
    ros::Publisher idle_state_pub = n.advertise<std_msgs::String>("Idle", 1);


    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 2, cmd_vel_get);
    ros::Subscriber goal_status_sub = n.subscribe("goal_status", 2, goal_status_get);
    ros::Rate loop_rate(500);
    ros::Time start(ros::Time::now());
    

#if TFOUTPUT
    tf::TransformBroadcaster br;
#endif

    int count = 0;
    tf::Vector3 displacement(0,0,0);
    double t_before = 0;
    double delta_t;
    
    tf::Vector3 origin(0,0,0);
    double phi = 0;

    bool ros_info_move_sent = false;
    

    // Quantities to fill in
    tf::Transform desired_pose(tf::Transform::getIdentity());

    ROS_INFO("Start to take off! Time Elapsed: %f s", (ros::Time::now()-start).toSec()); 

    bool taking_off  = false;
    bool take_off_finished = false;

    bool traveling = false;
    bool traveling_finished = false;

    while (ros::ok()) {
        

        double t = (ros::Time::now()-start).toSec();
        
        



        geometry_msgs::Twist velocity;
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0;
        geometry_msgs::Twist acceleration;
        acceleration.linear.x = acceleration.linear.y = acceleration.linear.z = 0;
        acceleration.angular.x = acceleration.angular.y = acceleration.angular.z = 0;


        std_msgs::String take_off_msg;
        std_msgs::String traveling_msg;
        std_msgs::String landing_msg;
        
         

        if (t <= 4)
        {
            //take off
          
            take_off_msg.data = "positive";
            take_off_state_pub.publish(take_off_msg); 
              
            displacement = tf::Vector3(0,0,t*5/4);
            desired_pose.setOrigin(origin+displacement);
                
            tf::Quaternion q;
            // q.setRPY(0,0,phi-t*PI/4);
            q.setRPY(0,0,0);

            desired_pose.setRotation(q);
            t_before = t;
            // ROS_INFO("%f", t );
                

        }else if(!goal_reached)
        { 
           
            
        take_off_msg.data = "succeeded";
        take_off_state_pub.publish(take_off_msg); 
        // Rate.sleep();
            
            
            
        traveling_msg.data = "positive";
        traveling_state_pub.publish(traveling_msg);
        // Rate.sleep();
              
            // flying towards goal
            // controlled by move_base through '/cmd_vel'
                if (!ros_info_move_sent)
                {
                // ROS_INFO("Moving towards goal! Time Elapsed: %f s", (ros::Time::now()-start).toSec()); 
                ros_info_move_sent = true;
                }
                
                delta_t = t - t_before;
                t_before = t;
                
                phi = phi + delta_t * vel_got.angular.z ;

                origin = origin +  delta_t * tf::Vector3(
                         vel_got.linear.x * cos(phi) - vel_got.linear.y * sin(phi), vel_got.linear.x * sin(phi) + vel_got.linear.y * cos(phi), 0);

               
                // velocity.linear.x = vel_got.linear.x * cos(phi) - vel_got.linear.y * sin(phi);
                // velocity.linear.y = vel_got.linear.x * sin(phi) + vel_got.linear.y * cos(phi);

                desired_pose.setOrigin(origin+displacement);
                                
                tf::Quaternion q;
                q.setRPY(0,0,phi);
                // count++;
                // std::cout<<"Desired Orientation" << count << std::endl;
                desired_pose.setRotation(q);
                
                // ROS_INFO("%f", t );




        } else if (goal_reached)
        {
            // landing
           
            traveling_msg.data = "succeeded";
            traveling_state_pub.publish(traveling_msg); 
    
            
            landing_msg.data = "positive";
            landing_state_pub.publish(landing_msg);


                double t_after_goal_reached = t - (time_goal_reached - start).toSec();
              
                // ROS_INFO("%f",  t_after_goal_reached);
                if (t_after_goal_reached < 1)
                {
                displacement = tf::Vector3(0,0, 5 - t_after_goal_reached *5 / 1);
                desired_pose.setOrigin(origin+displacement);
                
                tf::Quaternion q;
                q.setRPY(0,0,phi);
                desired_pose.setRotation(q);
                } else
                {
                   ROS_INFO("Mission succeed! Time Elapsed: %f s", t);
                   landing_msg.data = "succeeded";
                   landing_state_pub.publish(landing_msg); 
                //    ros::Duration(2).sleep();
                   return 0;
                }
                
        }







        // Publish
        trajectory_msgs::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.transforms[0].translation.x = desired_pose.getOrigin().x();
        msg.transforms[0].translation.y = desired_pose.getOrigin().y();
        msg.transforms[0].translation.z = desired_pose.getOrigin().z();
        msg.transforms[0].rotation.x = desired_pose.getRotation().getX();
        msg.transforms[0].rotation.y = desired_pose.getRotation().getY();
        msg.transforms[0].rotation.z = desired_pose.getRotation().getZ();
        msg.transforms[0].rotation.w = desired_pose.getRotation().getW();
        msg.velocities.resize(1);
        msg.velocities[0] = velocity;
        msg.accelerations.resize(1);
        msg.accelerations[0] = acceleration;
        desired_state_pub.publish(msg);

        // std::stringstream ss;
        // ss << "Trajectory Position"
        //    << " x:" << desired_pose.getOrigin().x()
        //    << " y:" << desired_pose.getOrigin().y()
        //    << " z:" << desired_pose.getOrigin().z();
        // ROS_INFO("%s", ss.str().c_str());

 #if TFOUTPUT
        br.sendTransform(tf::StampedTransform(desired_pose, ros::Time::now(),
                                              "world", "av-desired"));
 #endif

        ros::spinOnce();

        loop_rate.sleep();
        // ++count;
        
    }


    return 0;
}
