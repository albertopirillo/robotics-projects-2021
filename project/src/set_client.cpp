#include "ros/ros.h"
#include "project/Set_odom.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "set_client");
    if(argc != 4)
    {
        ROS_INFO("Usage: rosrun project set X Y THETA");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient set_client = n.serviceClient<project::Set_odom>("set_odom");
    project::Set_odom srv;
    srv.request.x = atoll(argv[1]);
    srv.request.y = atoll(argv[2]);
    srv.request.theta = atoll(argv[3]);

    if (set_client.call(srv))
    {
        ROS_INFO("Odometry has been set");
    }
    else
    {
        ROS_INFO("Failed to call service set_odom");
        return 1;
    }
    return 0;
}