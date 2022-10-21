#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <cmath>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "conv");

    double degrees = 90;
    double radians = (degrees * M_PI) / 180.0;
    const tf::Quaternion &q = tf::createQuaternionFromYaw(radians);
    ROS_INFO("X: %f", q.getX());
    ROS_INFO("Y: %f", q.getY());
    ROS_INFO("Z: %f", q.getZ());
    ROS_INFO("W: %f", q.getW());

    ros::spinOnce();
    return 0;
}