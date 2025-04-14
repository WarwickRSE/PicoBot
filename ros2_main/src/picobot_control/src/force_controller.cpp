#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "message_filters/subscriber.h"
#include "geometry_msgs/WrenchStamped.h"
#include <cstdlib>
#include <tf2_ros/static_transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include "picobot_control/ForceZConfig.h"

float desired_force = 3.0;
class ForceController{
    private:
        double  curr_force, error_fz, dFe, dZ;
        ros::Publisher pose_pub;
        ros::Subscriber wrench_sub;
        tf2::Transform transform_base_ee;
        geometry_msgs::TransformStamped transformStamped_base_to_end;
        geometry_msgs::TransformStamped transformStamped_goal;
        double Kp = 1.0;
        double Kd = 1.0;
        double Kf = 1000.0;
        double dt = 0.1;
        double prev_error_fz = 0.0;
        bool first_run = true;
        double alpha = 0.9;
        double smoothed_dFe = 0.0;
        geometry_msgs::PoseStamped pose_got;
        
    public:
    ForceController(ros::NodeHandle *nh){
      
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        //looking up transformation from base to end-effector of the robot
        try {
                transformStamped_base_to_end = tfBuffer.lookupTransform("iiwa_link_0", "tool_link_ee", ros::Time(0), ros::Duration(2));
            }
        catch (tf2::TransformException &ex) {
                ROS_WARN("%s",ex.what());    
        }
        
        std::cout << transformStamped_base_to_end << std::endl;
        pose_pub = nh->advertise<geometry_msgs::PoseStamped>("/iiwa/CartesianImpedance_trajectory_controller/reference_pose",1);
        wrench_sub = nh->subscribe("/cartesian_wrench_tool_biased",10, &ForceController::callback_controller, this);
}

    void callback_controller(const geometry_msgs::WrenchStamped& force_msg){
        
        curr_force = force_msg.wrench.force.z;
        // std::cout << "current_force" << curr_force << std::endl;
        error_fz = desired_force - abs(curr_force);
        // std::cout << "desired_force" << desired_force << std::endl;
        //  std::cout << "Fz error" << error_fz << std::endl;
        if (first_run){
            prev_error_fz = error_fz;
            first_run = false;
        }
        dFe = error_fz - prev_error_fz;
        smoothed_dFe = alpha*smoothed_dFe + (1-alpha)*dFe;
        prev_error_fz = error_fz;
        dZ = (Kp/Kf)*error_fz*dt + (Kd/Kf)*smoothed_dFe*dt;        
        // std::cout << "dZ" << dZ << std::endl;
        double max_dZ = 0.00025;  // Maximum allowed movement per iteration
        if (fabs(dZ) > max_dZ) {
            dZ = copysign(max_dZ, dZ);  // Clamp the displacement
            std::cout << "Clamped dZ" << dZ << std::endl;
        }
        geometry_msgs::PoseStamped pose = update_pose(dZ);
        pose_pub.publish(pose);
        
    }

    void callback_dyn_param(picobot_control::ForceZConfig &config, uint32_t level){
        desired_force = config.desired_force;
    }


    geometry_msgs::PoseStamped update_pose(double& dZ){
        transformStamped_goal.header.stamp = ros::Time::now();
        transformStamped_goal.header.frame_id = "tool_link_ee";
        transformStamped_goal.child_frame_id = "goal_point";
        // std::cout << "prev_z" << transformStamped_goal.transform.translation.z << std::endl;
        transformStamped_goal.transform.translation.x = 0.0;
        transformStamped_goal.transform.translation.y = 0.0;
        transformStamped_goal.transform.translation.z += dZ;
        transformStamped_goal.transform.rotation.x = 0.0;
        transformStamped_goal.transform.rotation.y = 0.0;
        transformStamped_goal.transform.rotation.z = 0.0;
        transformStamped_goal.transform.rotation.w = 1; 
        // std::cout << transformStamped_goal << std::endl;
        tf2::Vector3 translation_goal(transformStamped_goal.transform.translation.x,
                                        transformStamped_goal.transform.translation.y,
                                        transformStamped_goal.transform.translation.z);

        tf2::Quaternion quat_tf_goal;
        tf2::convert(transformStamped_goal.transform.rotation, quat_tf_goal);
        quat_tf_goal.normalize();
        tf2::Transform transform_goal(quat_tf_goal,translation_goal);

        tf2::Vector3 translation_base_ee(transformStamped_base_to_end.transform.translation.x,transformStamped_base_to_end.transform.translation.y,transformStamped_base_to_end.transform.translation.z);
        tf2::Quaternion quat_tf_base_end;
        tf2::convert(transformStamped_base_to_end.transform.rotation, quat_tf_base_end);
        quat_tf_base_end.normalize();
        tf2::Transform transform_base_ee(quat_tf_base_end,translation_base_ee);

        tf2:: Transform transformed_goal_base = transform_base_ee*transform_goal;

        geometry_msgs::TransformStamped static_transform_goal_base;
        static_transform_goal_base.header.stamp = ros::Time::now();
        static_transform_goal_base.header.frame_id = "world";
        static_transform_goal_base.child_frame_id = "goal_frame";
        static_transform_goal_base.transform = tf2::toMsg(transformed_goal_base);

        // check distance
        geometry_msgs::Vector3 goal_translation = static_transform_goal_base.transform.translation;
        geometry_msgs::Vector3 end_translation = transformStamped_base_to_end.transform.translation;
        double distance = sqrt(pow(goal_translation.x - end_translation.x,2)
                                + pow(goal_translation.y - end_translation.y,2)
                                + pow(goal_translation.z - end_translation.z,2));
        
       
        if (distance > 0.06) {
            // Perform action if the distance is more than 6 cm
            ROS_WARN("Distance from start is greater than 6 cm.");
            return pose_got; // previously saved pose goat
        }
        else {
            pose_got.header.frame_id="world";
            pose_got.header.stamp = ros::Time::now();
            pose_got.pose.position.x = static_transform_goal_base.transform.translation.x;
            pose_got.pose.position.y = static_transform_goal_base.transform.translation.y;
            pose_got.pose.position.z = static_transform_goal_base.transform.translation.z;
            pose_got.pose.orientation.x = static_transform_goal_base.transform.rotation.x;
            pose_got.pose.orientation.y = static_transform_goal_base.transform.rotation.y;
            pose_got.pose.orientation.z = static_transform_goal_base.transform.rotation.z;
            pose_got.pose.orientation.w = static_transform_goal_base.transform.rotation.w;
            // std::cout << pose_got << std::endl;

            return pose_got;
        }
        


    }
};
        
int main(int argc, char **argv)
{
    ros::init(argc,argv, "force_controller");
    ros::NodeHandle nh;
    dynamic_reconfigure::Server<picobot_control::ForceZConfig> server;
    dynamic_reconfigure::Server<picobot_control::ForceZConfig>::CallbackType f;
    ForceController fc = ForceController(&nh);
    f = boost::bind(&ForceController::callback_dyn_param, fc, _1, _2);
    server.setCallback(f);
    ros::spin();
}
