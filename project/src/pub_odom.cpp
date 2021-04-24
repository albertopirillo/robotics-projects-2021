#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "project/Odom_and_method.h"
#include "project/Initial_pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "data.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_broadcaster.h>

class pub_odom 
{

public:
    enum method {Euler, Runge_Kutta};
    project::Odom_and_method msg;
    
private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    int current_method;
    float x, y, theta;
    ros::Time current_time, last_time;

public:
    pub_odom()
    {
        sub = n.subscribe("/my_twist", 1000, &pub_odom::callback, this);
        pub = n.advertise<project::Odom_and_method>("/my_odom", 1000);
        n.getParam("/initial_x", x);  
        n.getParam("/initial_y", y);  
        n.getParam("/initial_theta", x);  
        current_time = ros::Time::now();
    }

    void callback(const geometry_msgs::TwistStamped::ConstPtr& twist) 
    {
        n.getParam("/params/method", current_method);
        if (current_method == method::Euler) compute_euler(twist);
        else compute_runge_kutta(twist);
    }

    void compute_euler(const geometry_msgs::TwistStamped::ConstPtr& twist) 
    {
        ROS_INFO("Computing odometry with Euler");
        msg.method.data = "Euler";

        current_time = twist->header.stamp;
        int Ts =  (current_time - last_time).toNSec();
        last_time = current_time;

        x = x + twist->twist.linear.x * Ts * cos(theta);
        y = y + twist->twist.linear.y * Ts * sin(theta);
        theta = theta + twist->twist.angular.z * Ts;
        publish_msg(twist);
    }

    void compute_runge_kutta(const geometry_msgs::TwistStamped::ConstPtr& twist)
    {
        ROS_INFO("Computing odometry with Runge-Kutta");
        msg.method.data = "Runge-Kutta";

        current_time = twist->header.stamp;
        int Ts =  (current_time - last_time).toNSec();
        last_time = current_time;

        x = x + twist->twist.linear.x * Ts * cos(theta + (twist->twist.angular.z * Ts / 2));
        x = x + twist->twist.linear.x * Ts * sin(theta + (twist->twist.angular.z * Ts / 2));
        theta = theta + twist->twist.angular.z * Ts;
        publish_msg(twist);
    }

    void publish_msg(const geometry_msgs::TwistStamped::ConstPtr& twist) 
    {
        msg.odom.pose.pose.position.x = x;
        msg.odom.pose.pose.position.y = y;
        msg.odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        msg.odom.header = twist->header;
        //msg.odom.child_frame_id = //TODO
        msg.odom.twist.twist = twist->twist;
        pub.publish(msg);  
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom");
    pub_odom my_pub_odom;
    ros::spin();
    return 0;
}