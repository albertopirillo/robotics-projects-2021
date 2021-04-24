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
        n.getParam("/initial_theta", theta);  
        current_time = ros::Time::now();
        last_time = ros::Time::now();
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

        current_time = ros::Time::now();
        int Ts_NSec =  (current_time - last_time).toNSec();
        float Ts_Sec = Ts_NSec / 1000000000.0;
        last_time = current_time;

        //With skid-steering approximate kinematics, vel_y is always 0
        float vel = twist->twist.linear.x; 
        x = x + vel * Ts_Sec * cos(theta);
        y = y + vel * Ts_Sec * sin(theta);
        theta = theta + twist->twist.angular.z * Ts_Sec;
        publish_msg(twist);
    }

    void compute_runge_kutta(const geometry_msgs::TwistStamped::ConstPtr& twist)
    {
        ROS_INFO("Computing odometry with Runge-Kutta");
        msg.method.data = "Runge-Kutta";

        current_time = ros::Time::now();
        int Ts_NSec =  (current_time - last_time).toNSec();
        float Ts_Sec = Ts_NSec / 1000000000.0;
        last_time = current_time;

        //TODO: probably twsit.angular.z is broken => debug with pritnf
        float argument = theta + ((twist->twist.angular.z * Ts_NSec) / 2.0);
        float vel = twist->twist.linear.x;
        x = x + vel * Ts_NSec * cos(argument);
        y = y + vel * Ts_NSec * sin(argument);
        theta = theta + twist->twist.angular.z * Ts_NSec;
        publish_msg(twist);
    }

    void publish_msg(const geometry_msgs::TwistStamped::ConstPtr& twist) 
    {
        msg.odom.pose.pose.position.x = x;
        msg.odom.pose.pose.position.y = y;
        msg.odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

        msg.odom.header.stamp.sec = twist->header.stamp.sec;
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