#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "eye_in_hand");
    ros::NodeHandle node;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    ros::Rate rate(10.0);
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;


    //looking up transformation from base to end-effector of the robot
    geometry_msgs::TransformStamped transformStamped_base_to_end;
    while (node.ok()){
    
        try {
            transformStamped_base_to_end = tfBuffer.lookupTransform("iiwa_link_0", "iiwa_link_ee", ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        
        }
    tf2::Vector3 translation_base_ee(transformStamped_base_to_end.transform.translation.x,transformStamped_base_to_end.transform.translation.y,transformStamped_base_to_end.transform.translation.z);
    tf2::Quaternion quat_tf_base_end;
    tf2::convert(transformStamped_base_to_end.transform.rotation, quat_tf_base_end);
    quat_tf_base_end.normalize();
    tf2::Transform transform_base_ee(quat_tf_base_end,translation_base_ee);
    //std::cout<< transformStamped_base_to_end << std::endl;


    // fixing transformation of end-effector to the optical frame after calibration
    // with rs_camera mount only
    /*geometry_msgs::TransformStamped transformStamped_calib;
    transformStamped_calib.header.stamp = ros::Time::now();
    transformStamped_calib.header.frame_id = "iiwa_link_ee";
    transformStamped_calib.child_frame_id = "camera_color_optical_frame";
    transformStamped_calib.transform.translation.x = -0.0266552;
    transformStamped_calib.transform.translation.y = -0.05815;
    transformStamped_calib.transform.translation.z = 0.02595;
    transformStamped_calib.transform.rotation.x = -0.0116221;
    transformStamped_calib.transform.rotation.y = -0.00501024;
    transformStamped_calib.transform.rotation.z = 0.00715;
    transformStamped_calib.transform.rotation.w = 0.9998;  */

    
    // rs_camera_mount_with_tool
    geometry_msgs::TransformStamped transformStamped_calib;
    transformStamped_calib.header.stamp = ros::Time::now();
    transformStamped_calib.header.frame_id = "iiwa_link_ee";
    transformStamped_calib.child_frame_id = "camera_color_optical_frame";
    transformStamped_calib.transform.translation.x = -0.0284;//-0.0265; //-0.0297205;//-0.0245998//-0.0384;
    transformStamped_calib.transform.translation.y = -0.072;//-0.0703218;//-0.058607;//-0.072;
    transformStamped_calib.transform.translation.z = 0.023044;
    transformStamped_calib.transform.rotation.x = 0.00493;
    transformStamped_calib.transform.rotation.y = -0.004712;
    transformStamped_calib.transform.rotation.z = 0.0177509;
    transformStamped_calib.transform.rotation.w = 0.999819;  


    tf2::Vector3 translation_calib(transformStamped_calib.transform.translation.x,
                                    transformStamped_calib.transform.translation.y,
                                    transformStamped_calib.transform.translation.z );
    tf2::Quaternion quat_tf_calib;
    tf2::convert(transformStamped_calib.transform.rotation, quat_tf_calib);
    quat_tf_calib.normalize();
    tf2::Transform transform_calib(quat_tf_calib,translation_calib);

    tf2:: Transform transformed_color_base = transform_base_ee*transform_calib;
    geometry_msgs::TransformStamped static_transform_color_base;
    static_transform_color_base.header.stamp = ros::Time::now();
    static_transform_color_base.header.frame_id = "iiwa_link_0";
    static_transform_color_base.child_frame_id = "camera_color_optical_frame";
    static_transform_color_base.transform = tf2::toMsg(transformed_color_base);
    
    //static_transform_depth_base.transform.rotation = transformStamped_calib.transform.rotation*static_transform_color_depth.transform.rotation
    //static_transform_depth_base.transform.rotation.normalize();
    static_broadcaster.sendTransform(static_transform_color_base);



    // fixing transformation of depth to the optical frame using camera TF;
    geometry_msgs::TransformStamped static_transform_color_depth;
    static_transform_color_depth.header.stamp = ros::Time::now();
    static_transform_color_depth.header.frame_id = "camera_depth_optical_frame";
    static_transform_color_depth.child_frame_id = "camera_color_optical_frame";
    static_transform_color_depth.transform.translation.x = -0.015;
    static_transform_color_depth.transform.translation.y = -0;
    static_transform_color_depth.transform.translation.z = -0.001;
    static_transform_color_depth.transform.rotation.x = 0.002;
    static_transform_color_depth.transform.rotation.y = -0.002;
    static_transform_color_depth.transform.rotation.z = 0.015;
    static_transform_color_depth.transform.rotation.w = 1.0;

    tf2::Vector3 translation_color_depth(static_transform_color_depth.transform.translation.x,
                                        static_transform_color_depth.transform.translation.y,
                                        static_transform_color_depth.transform.translation.z);
    tf2::Quaternion quat_tf_color_depth;
    tf2::convert(static_transform_color_depth.transform.rotation, quat_tf_color_depth);
    quat_tf_color_depth.normalize();
    tf2::Transform transform_color_depth(quat_tf_color_depth,translation_color_depth);

    // Applying transformation from end-effector to the depth frame.
    tf2::Transform tranformed_depth_ee = transform_calib*transform_color_depth;
    geometry_msgs::TransformStamped static_transform_depth_ee;
    static_transform_depth_ee.header.stamp = ros::Time::now();
    static_transform_depth_ee.header.frame_id = "iiwa_link_ee";
    static_transform_depth_ee.child_frame_id = "camera_depth_optical_frame";
    static_transform_depth_ee.transform = tf2::toMsg(tranformed_depth_ee);


    //std::cout<<static_transform_depth_ee.transform.translation.x << "--" << static_transform_depth_ee.transform.translation.y << "--" <<
    //          static_transform_depth_ee.transform.translation.z << std::endl;
    //std::cout<<static_transform_depth_ee.transform.rotation.x << "--" << static_transform_depth_ee.transform.rotation.y << "--" <<
      //        static_transform_depth_ee.transform.rotation.z << "--"<< static_transform_depth_ee.transform.rotation.w << std::endl;


    // Applying transformation from robot base to the depth frame.
    tf2::Transform transformed_base_depth = transform_base_ee*tranformed_depth_ee;
    geometry_msgs::TransformStamped static_transform_depth_base;
    static_transform_depth_base.header.stamp = ros::Time::now();
    static_transform_depth_base.header.frame_id = "iiwa_link_0";
    static_transform_depth_base.child_frame_id = "camera_depth_optical_frame";
    static_transform_depth_base.transform = tf2::toMsg(transformed_base_depth);
    
    //static_transform_depth_base.transform.rotation = transformStamped_calib.transform.rotation*static_transform_color_depth.transform.rotation
    //static_transform_depth_base.transform.rotation.normalize();
    static_broadcaster.sendTransform(static_transform_depth_base);
    // //ros::spin();
     rate.sleep();
    }
    return 0;
};
