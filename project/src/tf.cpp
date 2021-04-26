#include "ros/ros.h"
#include "project/Odom_and_method.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class tf_sub_pub
{
private:
    ros::NodeHandle n;
    tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped msg;
    ros::Subscriber sub;

public:
    tf_sub_pub()
    {
        sub = n.subscribe("/my_odom", 1000, &tf_sub_pub::callback, this);
    }

    void callback(const project::Odom_and_method::ConstPtr& my_odom)
    {
        msg.header.stamp = my_odom->odom.header.stamp;
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link"; 
        msg.transform.translation.x = my_odom->odom.pose.pose.position.x;
        msg.transform.translation.y = my_odom->odom.pose.pose.position.y;
        msg.transform.translation.z = my_odom->odom.pose.pose.position.z;
        msg.transform.rotation.x = my_odom->odom.pose.pose.orientation.x;
        msg.transform.rotation.y = my_odom->odom.pose.pose.orientation.y;
        msg.transform.rotation.z = my_odom->odom.pose.pose.orientation.z;
        msg.transform.rotation.w = my_odom->odom.pose.pose.orientation.w;
        br.sendTransform(msg);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_node");
    tf_sub_pub my_tf_sub_pub;
    ros::spin();
    return 0;
}