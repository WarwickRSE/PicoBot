#include "ros/ros.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_target_pose");
    ros::NodeHandle nh_send;
    ros::Publisher pub_send = nh_send.advertise<geometry_msgs::PoseStamped>("cartesian_trajectory_generator/new_goal",1);
    geometry_msgs::PoseArray::ConstPtr pose_array_dZ = ros::topic::waitForMessage<geometry_msgs::PoseArray>("/tf_array_out_dZ");
    geometry_msgs::PoseStamped point_now_dZ;
    point_now_dZ.header.frame_id = "world";
    point_now_dZ.header.stamp = ros::Time::now();
    point_now_dZ.pose = pose_array_dZ->poses[0];
    pub_send.publish(point_now_dZ);
    //  ros::Duration(20.0).sleep();
    // ros::spin();
    return 0;
}
