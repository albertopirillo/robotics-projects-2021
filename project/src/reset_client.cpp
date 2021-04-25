#include "ros/ros.h"
#include "project/Reset_odom.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "reset_client");
    if(argc != 1)
    {
        ROS_INFO("Usage: rosrun project reset");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient reset_client = n.serviceClient<project::Reset_odom>("reset_odom");
    project::Reset_odom srv;
    if (reset_client.call(srv))
    {
        ROS_INFO("Odometry has been reset");
    }
    else
    {
        ROS_INFO("Failed to call service reset_odom");
        return 1;
    }
    return 0;
}