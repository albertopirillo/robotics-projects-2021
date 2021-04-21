#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "geometry_msgs/TwistStamped.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>

//Know parameters, in meters
#define RADIUS 0.1575
#define REAL_BL 0.583
#define APPARENT_BL REAL_BL * 1.78125 
//So the apparent baseline is about 1.038 meters

//Know parameters, dimensionless
#define GEAR_RATIO 1/38.125

//Global variables


//Prints all Motor speeds synchronized
void print_rpm_callback(const robotics_hw1::MotorSpeed::ConstPtr& motorFR,
              const robotics_hw1::MotorSpeed::ConstPtr& motorRR,
              const robotics_hw1::MotorSpeed::ConstPtr& motorRL,
              const robotics_hw1::MotorSpeed::ConstPtr& motorFL) {
                
    ROS_INFO("- Front right speed is: [%f]", motorFR->rpm);
    ROS_INFO("- Rear  right speed is: [%f]", motorRR->rpm);
    ROS_INFO("- Rear  left  speed is: [%f]", motorRL->rpm);
    ROS_INFO("- Front left  speed is: [%f]", motorFL->rpm);
    ROS_INFO("---------------------------------------");
}

//Estimate linear and angular velocity of the robot from motor speeds
void estimate_callback(const robotics_hw1::MotorSpeed::ConstPtr& motorFR,
              const robotics_hw1::MotorSpeed::ConstPtr& motorRR,
              const robotics_hw1::MotorSpeed::ConstPtr& motorRL,
              const robotics_hw1::MotorSpeed::ConstPtr& motorFL,
              ros::Publisher pub) {

    float right_rpm =  (motorFR->rpm + motorRR->rpm) / 2;
    float left_rpm  = -(motorFL->rpm + motorRL->rpm) / 2;

    float scaling_factor = 2 * M_PI * RADIUS * GEAR_RATIO / 60 ;
    float vel_r = right_rpm * scaling_factor;
    float vel_l = left_rpm  * scaling_factor;

    float vel_x = (vel_r + vel_l) / 2;
    float vel_y = 0;

    float y0 = APPARENT_BL / 2;
    float w_z = (vel_r - vel_l) / (2 * y0);
    
    geometry_msgs::TwistStamped msg;
    msg.twist.linear.x = vel_x;
    msg.twist.linear.y = 0;
    msg.twist.angular.z = w_z;
    msg.header.stamp = motorFR->header.stamp;
    pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "repub");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::TwistStamped>("estimate", 1000);

    //Synchronized Subscribers
    message_filters::Subscriber<robotics_hw1::MotorSpeed> motorFR(n, "/motor_speed_fr", 1);
    message_filters::Subscriber<robotics_hw1::MotorSpeed> motorRR(n, "/motor_speed_rr", 1);  
    message_filters::Subscriber<robotics_hw1::MotorSpeed> motorRL(n, "/motor_speed_rl", 1);  
    message_filters::Subscriber<robotics_hw1::MotorSpeed> motorFL(n, "/motor_speed_fl", 1);    

    //Define SyncPolicy
    typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed,
     robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;
    
    //Assign Subscribers and callback function
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), motorFR, motorRR, motorRL, motorFL);
    sync.registerCallback(boost::bind(&estimate_callback, _1, _2, _3, _4, pub));

    ros::spin();

    return 0;
}