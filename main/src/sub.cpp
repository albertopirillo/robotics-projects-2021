#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "repub");
    ros::NodeHandle n;

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
    sync.registerCallback(boost::bind(&print_rpm_callback, _1, _2, _3, _4));

    ros::spin();

    return 0;
}