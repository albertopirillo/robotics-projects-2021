#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include "geometry_msgs/TwistStamped.h"
#include "data.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cmath>

//Define SyncPolicy
typedef message_filters::sync_policies::ApproximateTime<robotics_hw1::MotorSpeed,
    robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;

class pub_twist
{
public:
    geometry_msgs::TwistStamped msg;

private:
    ros::NodeHandle n;
    ros::Publisher pub;
    //Synchronized subscribers
    message_filters::Subscriber<robotics_hw1::MotorSpeed> motorFR;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> motorRR; 
    message_filters::Subscriber<robotics_hw1::MotorSpeed> motorRL;
    message_filters::Subscriber<robotics_hw1::MotorSpeed> motorFL; 
    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync;

public:
    pub_twist() 
    {
        pub = n.advertise<geometry_msgs::TwistStamped>("my_twist", 1000);
        motorFR.subscribe(n, "/motor_speed_fr", 1);
        motorRR.subscribe(n, "/motor_speed_rr", 1);
        motorRL.subscribe(n, "/motor_speed_rl", 1);
        motorFL.subscribe(n, "/motor_speed_fl", 1);
        sync.reset(new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(10), motorFR, motorRR, motorRL, motorFL));
        sync->registerCallback(boost::bind(&pub_twist::callback, this, _1, _2, _3, _4));
    }


    void callback(const robotics_hw1::MotorSpeed::ConstPtr& motorFr,
              const robotics_hw1::MotorSpeed::ConstPtr& motorRr,
              const robotics_hw1::MotorSpeed::ConstPtr& motorRl,
              const robotics_hw1::MotorSpeed::ConstPtr& motorFl)
    {
        //Average rps of the two pairs of motors
        double right_rpm = (motorFr->rpm + motorRr->rpm) / 2;
        double left_rpm  = -(motorFl->rpm + motorRl->rpm) / 2;

        //Compute linear velocity
        float scaling_factor = 2 * M_PI * RADIUS * GEAR_RATIO / 60 ;
        double vel_r = right_rpm * scaling_factor;
        double vel_l = left_rpm  * scaling_factor;

        double vel_x = (vel_r + vel_l) / 2;
        //double vel_y = 0;

        //Compute angular velocity
        double y0 = APPARENT_BL / 2;
        double w_z = (vel_r - vel_l) / (2 * y0);

        //Publish the message
        msg.header.stamp = motorFr->header.stamp;
        msg.header.frame_id = "odom";
        msg.twist.linear.x = vel_x;
        // msg.twist.linear.y = 0;
        // msg.twist.linear.z = 0;
        // msg.twist.angular.x = 0;
        // msg.twist.angular.y = 0;
        msg.twist.angular.z = w_z;
        pub.publish(msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "twist");
    pub_twist my_pub_twist;
    ros::spin();
    return 0;
}
