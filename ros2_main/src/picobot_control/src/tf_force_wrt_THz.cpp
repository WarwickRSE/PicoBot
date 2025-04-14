#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class TFForceWrtTHz{

public:
    TFForceWrtTHz(ros::NodeHandle nh):nh_(nh), tf_listener_(tf_buffer_)
    {
        wrench_sub_ = nh_.subscribe("/cartesian_wrench_tool_biased", 1, &TFForceWrtTHz::wrenchCallback, this);
        wrench_pub_ts = nh_.advertise<geometry_msgs::WrenchStamped>("/cartesian_wrench_tool_ts", 1);
        eef_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("/tool_link_ee_pose", 1);
    }

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
    {   
        geometry_msgs::WrenchStamped wrench_at_ten_hz;
        wrench_at_ten_hz.header.frame_id = msg->header.frame_id;
        wrench_at_ten_hz.header.stamp = ros::Time::now();
        wrench_at_ten_hz.wrench = msg->wrench;

        geometry_msgs::TransformStamped transformation_base_ee;
        try {
            transformation_base_ee = tf_buffer_.lookupTransform("iiwa_link_0", "tool_link_ee", ros::Time(0));
            eef_pose_.publish(transformation_base_ee);
            wrench_pub_ts.publish(wrench_at_ten_hz);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            return;
        }
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    ros::Subscriber wrench_sub_;
    ros::Publisher wrench_pub_ts;
    ros::Publisher eef_pose_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_force_wrt_THz");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    TFForceWrtTHz tf_force_wrt_THz(nh);
    while (ros::ok()) {
        ros::spinOnce(); // Process callbacks
        loop_rate.sleep(); // Sleep to maintain the 10 Hz rate
    }
    
    return 0;
}