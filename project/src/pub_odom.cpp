#include "ros/ros.h"
#include "data.h"
#include "robotics_hw1/MotorSpeed.h"
#include "project/Odom_and_method.h"
#include "project/Initial_pose.h"
#include "geometry_msgs/TwistStamped.h"
#include "project/Reset_odom.h"
#include "project/Set_odom.h"
#include <math.h>
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
    ros::ServiceServer reset_srv;
    ros::ServiceServer set_srv;
    int current_method;
    float x, y, theta;
    ros::Time current_time, last_time;

public:

    pub_odom()
    {
        n.getParam("/initial_x", x);  
        n.getParam("/initial_y", y);  
        n.getParam("/initial_theta", theta); 
        sub = n.subscribe("/my_twist", 1000, &pub_odom::callback, this);
        pub = n.advertise<project::Odom_and_method>("/my_odom", 1000);
        reset_srv = n.advertiseService("reset_odom", &pub_odom::reset, this);
        set_srv = n.advertiseService("set_odom", &pub_odom::set, this);
        current_time = ros::Time::now();
        last_time = ros::Time::now();
    }
    
    bool reset(project::Reset_odom::Request &req, project::Reset_odom::Response &res)
    {
        x = 0;
        y = 0;
        theta = 0; //TODO: also theta?
        ROS_INFO("Odom has been reset to (0,0)");
        return true;
    }

    bool set(project::Set_odom::Request &req, project::Set_odom::Response &res)
    {
        x = req.x;
        y = req.y;
        theta = req.theta;
        ROS_INFO("Odom has been set to ([%f],[%f],[%f])", req.x, req.y, req.theta);
        return true;
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
        msg.method.data = "euler";

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
        msg.method.data = "rk";

        current_time = ros::Time::now();
        int Ts_NSec =  (current_time - last_time).toNSec();
        float Ts_Sec = Ts_NSec / 1000000000.0;
        last_time = current_time;

        //TODO: probably twsit.angular.z is broken => debug with pritnf
        float argument = theta + ((twist->twist.angular.z * Ts_Sec) / 2.0);
        float vel = twist->twist.linear.x;
        x = x + vel * Ts_Sec * cos(argument);
        y = y + vel * Ts_Sec * sin(argument);
        theta = theta + twist->twist.angular.z * Ts_Sec;
        publish_msg(twist);
    }

    void publish_msg(const geometry_msgs::TwistStamped::ConstPtr& twist) 
    {
        msg.odom.header.stamp = twist->header.stamp;
        msg.odom.header.seq = twist->header.seq;
        msg.odom.header.frame_id = "odom";
        msg.odom.child_frame_id = "base_link";
   
        msg.odom.pose.pose.position.x = x;
        msg.odom.pose.pose.position.y = y;
        msg.odom.pose.pose.position.z = 0;
        msg.odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
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