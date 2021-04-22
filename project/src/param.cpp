#include "ros/ros.h"
#include <dynamic_reconfigure/server.h>
#include <project/parametersConfig.h>
#include <std_msgs/Float64.h>

void callback(project::parametersConfig &config, uint32_t bitmask_level)
{
    ROS_INFO("Reconfigure request: %d", config.method_param);
    ROS_INFO ("%d",bitmask_level);
}

//TODO: initialize parameter into launch file
int main(int argc, char** argv)
{
    ros::init(argc, argv, "param");
    ros::NodeHandle n;

    //Publish initial parameters (no dynamic reconfigure needed)
    ros::Publisher pub_x = n.advertise<std_msgs::Float64>("initial_x", 1000);
    ros::Publisher pub_y = n.advertise<std_msgs::Float64>("initial_y", 1000);
    ros::Publisher pub_theta = n.advertise<std_msgs::Float64>("initial_theta", 1000);

    //Create the server and specify callback function type
    dynamic_reconfigure::Server<project::parametersConfig> server;
    dynamic_reconfigure::Server<project::parametersConfig>::CallbackType f;

    //Bind and set callback function
    f = boost::bind(&callback, _1, _2);
    server.setCallback(f);

    ros::spin();
    return 0;
}