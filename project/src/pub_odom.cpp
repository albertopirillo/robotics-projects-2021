#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "project/Odom_and_method.h"
#include "data.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <std_msgs/Float64.h>

//Global scope
enum method {Euler, Runge_Kutta};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    ros::NodeHandle n;
    ros::Publisher pub_odom = n.advertise<project::Odom_and_method>("my_odom", 1000);

    //Get initial parameters
    int current_method;
    float initial_x, initial_y, initial_theta;
    n.getParam("/initial_x", initial_x); 
    n.getParam("/initial_y", initial_y); 
    n.getParam("/initial_theta", initial_theta); 
    n.getParam("/param_method", current_method);

    while (ros::ok()) 
    {
        n.getParam("/param_method", current_method);
        if (current_method == method::Euler) ;//Euler
        else ;//Runge-Kutta

        //TODO: spin?
    }

    return 0;
}