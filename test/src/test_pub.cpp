#include "ros/ros.h"
#include "test/Num.h"

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "test_pub");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<test::Num>("test_num", 1000);

    ros::Rate loop_rate(1000);

    while (ros::ok()){
        static int i = 0;
        i = (i+1)%1000;
        test::Num msg;
        msg.num = i;
        pub.publish (msg);

        ros::spinOnce();
        loop_rate.sleep();
  	}
  	return 0;
}
