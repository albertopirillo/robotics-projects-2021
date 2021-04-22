#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "project/Odom_and_method.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include "data.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<project::Odom_and_method>("my_odom", 1000);

    return 0;
}