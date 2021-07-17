#include <ros/ros.h>
#include <std_msgs/String.h>
#include "std_msgs/String.h"
using  std::__cxx11::string;

string state;
bool ready = false;
bool taking_off = false;
bool traveling = false;
bool landing = false;
bool idling = false;

class TakeOff
{
private:
    ros::NodeHandle n;
    ros::Subscriber sub_;
public:
    string take_off_flag;
    void callback(const std_msgs::String::ConstPtr& msg)
    {
        if (state == "TAKE_OFF")
        {
            take_off_flag  =  msg->data;
        }
    }
    void init(const ros::NodeHandle  &n1)
    {
        n = n1;
        sub_ = n.subscribe("/TakeOff",  1,  &TakeOff::callback, this);
    }
    string execute()
    {
        state="TAKE_OFF";
        // ros::Rate loop_rate(5);
        while (ros::ok())
        {
            ros::spinOnce();
            // loop_rate.sleep();
            if (take_off_flag == "succeeded")
            {
                ROS_INFO("Reached desired height!");
                take_off_flag = "";
                return "succeeded";
            }
            else if (take_off_flag == "positive" && !taking_off)
            {
                ROS_INFO("Taking off!");
                taking_off = true;
            }
            else if (!ready)
            {
                ROS_INFO("Ready to take off!");
                ready = true;
            }
            return "";
        }
    } 
};

class Traveling
{
private:
    ros::NodeHandle n;
    ros::Subscriber sub_;
public:
    string Traveling_flag;
    void callback(const std_msgs::String::ConstPtr& msg)
    {
        if (state == "TRAVELING")
        {
            Traveling_flag = msg->data;
        }
    }
    void init(const ros::NodeHandle  &n1)
    {
        n = n1;
        sub_ = n.subscribe("/Traveling",  1,  &Traveling::callback, this);
    }
    string execute()
    {
        state = "TRAVELING";
        // ros::Rate loop_rate(5);
        while (ros::ok())
        {
            ros::spinOnce();
            // loop_rate.sleep();
            if (Traveling_flag == "succeeded")
            {
                // ROS_INFO("Ready for landing!");
                Traveling_flag = "";
                return "succeeded";
            }
            else if (Traveling_flag == "positive" && !traveling) 
            {
                ROS_INFO("Traveling!");
                traveling = true;
            }
        }
        return "";
    } 
};

class Landing
{
private:
    ros::NodeHandle n;
    ros::Subscriber sub_;
public:
    string Landing_flag;
    void callback(const std_msgs::String::ConstPtr& msg)
    {
        if (state == "LANDING")
        {
            Landing_flag = msg->data;
        }
    }
    void init(const ros::NodeHandle  &n1)
    {
        n = n1;
        sub_ = n.subscribe("/Landing",  1,  &Landing::callback, this);
    }
    string execute()
    {
        state = "LANDING";
        // ros::Rate loop_rate(5);
        while (ros::ok())
        {
            ros::spinOnce();
            // loop_rate.sleep();
            if (Landing_flag == "succeeded")
            {
                ROS_INFO("Mission succeeded!");
                Landing_flag = "";
                return "succeeded";
            }
            else if (Landing_flag == "positive" && !landing)
            {
                ROS_INFO("Landing!");
                landing = true;
            }
        }
        return "";
    } 
};

class Idle
{
private:
    ros::NodeHandle n;
    ros::Subscriber sub_;
public:
    string idle_flag;
    void callback(const std_msgs::String::ConstPtr& msg)
    {
        if (state == "IDLE")
        {
            idle_flag =  msg->data;
        }
    }
    void init(const ros::NodeHandle  &n1)
    {
        n = n1;
        sub_ = n.subscribe("/Idle",  1,  &Idle::callback, this);
    }
    string execute()
    {
        state="IDLE";
        // ros::Rate loop_rate(5);
        while (ros::ok())
        {
            ros::spinOnce();
            // loop_rate.sleep();
            if (idle_flag == "goal received")
            {
                ROS_INFO("Goal received!");
                idle_flag = "";
                return "goal received";
            }
            else if (!idling)
            {
                ROS_INFO("Idling!");
                idling = true;
            }
            
        }
        return "";
    } 
};

void work(const ros::NodeHandle &n)
{
    static TakeOff take_off;
    take_off.init(n);

    static Traveling traveling;
    traveling.init(n);

    static Landing landing;
    landing.init(n);

    static Idle idle;
    idle.init(n);

    while (ros::ok())
    {
        string res;
        if (state == "TAKE_OFF")
        {
            res = take_off.execute();
            if (res == "succeeded") state = "TRAVELING";
        }
        else if (state == "TRAVELING")
        {
            res = traveling.execute();
            if (res == "succeeded") state = "LANDING";
        }
        else if (state == "LANDING")
        {
            res = landing.execute();
            if (res == "succeeded") state = "IDLE";
        }
        else if (state == "IDLE")
         {
            res = idle.execute();
            if (res == "goal received") state = "TAKE_OFF";
        }
    }
}

int main(int argc, char **argv)
{
    // signal(SIGINT, exit);
    // signal(SIGTERM, exit);  

    ros::init(argc, argv, "state_machine");  
    ros::NodeHandle n;

    state="TAKE_OFF";
    work(n);

    return 0;
}
