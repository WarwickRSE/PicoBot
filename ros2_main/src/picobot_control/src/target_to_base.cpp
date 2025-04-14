#include "ros/ros.h"
#include <geometry_msgs/Point.h>
#include "geometry_msgs/PoseArray.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class PoseDrawer
{
public:
    PoseDrawer() :
    tf2_(buffer_),  target_frame_("iiwa_link_0"),
    tf2_filter_(point_sub_, buffer_, target_frame_, 10, 0)
  {
    // point_sub_.subscribe(n_, "pose_output", 10);
    point_sub_.subscribe(n_, "final_pose_output", 10);
    tf2_filter_.registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
    
  }

  void msgCallback(const geometry_msgs::PoseArrayConstPtr& point_ptr) 
  {
    //std::cout<<"Im in"<<std::endl;
    geometry_msgs::PoseArray tf_array_out;
    tf_array_out.header.stamp = ros::Time::now();
    tf_array_out.header.frame_id = "iiwa_link_0";
    geometry_msgs::PoseStamped tf_output_pose;
    tf_output_pose.header.stamp = ros::Time::now();
    tf_output_pose.header.frame_id = "iiwa_link_0";
    geometry_msgs::PoseStamped point_now;
    point_now.header.stamp = ros::Time::now();
    //point_now.header.frame_id = "camera_image_link";
    point_now.header.frame_id = "camera_color_optical_frame";
    //std::cout<<point_ptr->poses.size()<< std::endl;//

    int array_size = point_ptr->poses.size();
    for(int i = 0; i < array_size; i++) //array_size
      { 
        point_now.pose.position.x = point_ptr->poses[i].position.x;
        point_now.pose.position.y = point_ptr->poses[i].position.y;
        point_now.pose.position.z = point_ptr->poses[i].position.z;
        point_now.pose.orientation.x = point_ptr->poses[i].orientation.x;
        point_now.pose.orientation.y = point_ptr->poses[i].orientation.y;
        point_now.pose.orientation.z = point_ptr->poses[i].orientation.z;
        point_now.pose.orientation.w = point_ptr->poses[i].orientation.w;
        //std::cout<<point_now<< std::endl;
        try 
          {
          buffer_.transform(point_now, tf_output_pose, target_frame_);
          
          //std::cout<<tf_output_pose<<std::endl;
          //ROS_INFO(tf_output_pose
                  //tf_output_pose.point.x,
                  //tf_output_pose.point.y,
                  //tf_output_pose.point.z);
          }
        catch (tf2::TransformException &ex) 
        {
        ROS_WARN("Failure %s\n", ex.what()); //Print exception which was caught
        }

        // To complete dZ distance from the target point
        geometry_msgs::Quaternion quat_msg;
        quat_msg.w = tf_output_pose.pose.orientation.w;
        quat_msg.x = tf_output_pose.pose.orientation.x;
        quat_msg.y = tf_output_pose.pose.orientation.y;
        quat_msg.z = tf_output_pose.pose.orientation.z;
        tf2::Quaternion q_tf_dz;
        tf2::convert(quat_msg,q_tf_dz);
        
        tf2::Vector3 translation_tp(tf_output_pose.pose.position.x,tf_output_pose.pose.position.y,tf_output_pose.pose.position.z);
        tf2::Transform transform_tp(q_tf_dz, translation_tp);

        geometry_msgs::Quaternion quat_msg_;
        quat_msg_.w = 1;quat_msg_.x = 0;quat_msg_.y = 0;quat_msg_.z = 0;
        tf2::Quaternion q_tf_dz_;
        tf2::convert(quat_msg_,q_tf_dz_);
        // std::cout << q_tf_dz_.w() << q_tf_dz_.x() << q_tf_dz_.y() << q_tf_dz_.z() << std::endl;
        tf2::Vector3 translation_tp_(0,0,-0.01);
        tf2::Transform transform_tp_(q_tf_dz_, translation_tp_);

        // Apply transformation
        tf2:: Transform transformed_tp_cam = transform_tp*transform_tp_;
        geometry_msgs::TransformStamped static_transform_tp_cam;
        static_transform_tp_cam.header.stamp = ros::Time::now();
        static_transform_tp_cam.header.frame_id = "world";
        static_transform_tp_cam.child_frame_id = "goal_frame";
        static_transform_tp_cam.transform = tf2::toMsg(transformed_tp_cam);
        tf_output_pose.pose.position.x = static_transform_tp_cam.transform.translation.x;
        tf_output_pose.pose.position.y = static_transform_tp_cam.transform.translation.y;
        tf_output_pose.pose.position.z = static_transform_tp_cam.transform.translation.z;
        tf_output_pose.pose.orientation.x = static_transform_tp_cam.transform.rotation.x;
        tf_output_pose.pose.orientation.y = static_transform_tp_cam.transform.rotation.y;
        tf_output_pose.pose.orientation.z = static_transform_tp_cam.transform.rotation.z;
        tf_output_pose.pose.orientation.w = static_transform_tp_cam.transform.rotation.w;

        // push in array
        //std::cout<<tf_output_pose<< std::endl;
        

        tf_array_out.poses.push_back(tf_output_pose.pose);
        // std::cout<<"value is"<<tf_array_out.poses[0]<<std::endl;
      }

      

  pub.publish(tf_array_out);
 }

private:
  std::string target_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tf2_;
  ros::NodeHandle n_;
  message_filters::Subscriber<geometry_msgs::PoseArray> point_sub_;
  tf2_ros::MessageFilter<geometry_msgs::PoseArray> tf2_filter_;
  ros::Publisher pub = n_.advertise<geometry_msgs::PoseArray>("tf_array_out_dZ", 10);
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "target_to_base"); //Init ROS
  PoseDrawer pd; //Construct class
  
  ros::spin(); // Run until interupted 
  return 0;
};
