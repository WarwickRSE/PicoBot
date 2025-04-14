#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class StampedWrenchTransformer {
public:
    StampedWrenchTransformer(ros::NodeHandle& nh)
        : nh_(nh), rate1_(100), rate2_(10) // Set the desired publishing rates
    {
        // Initialize subscribers
        wrench_sub_ = nh_.subscribe("cartesian_wrench", 1, &StampedWrenchTransformer::wrenchCallback, this);

        // Initialize publishers
        wrench_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("cartesian_wrench_tool", 1);
        wrench_pub_ts = nh_.advertise<geometry_msgs::WrenchStamped>("cartesian_wrench_tool_ts", 1);
        eef_pose_ = nh_.advertise<geometry_msgs::TransformStamped>("/tool_link_ee_pose", 1);

        // Initialize TF2 listener
        tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);

        // Set up the ROS timers for the desired publishing rates
        timer1_ = nh_.createTimer(ros::Duration(1.0 / rate1_), &StampedWrenchTransformer::timerCallback1, this);
        timer2_ = nh_.createTimer(ros::Duration(1.0 / rate2_), &StampedWrenchTransformer::timerCallback2, this);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber wrench_sub_;
    ros::Publisher wrench_pub_;
    ros::Publisher wrench_pub_ts;
    ros::Publisher eef_pose_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener* tf_listener_;
    ros::Timer timer1_;
    ros::Timer timer2_;
    double rate1_;
    double rate2_;
    geometry_msgs::WrenchStamped latest_wrench_;

    void wrenchCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
        // Store the latest stamped wrench message
        latest_wrench_ = *msg;
    }

    void timerCallback1(const ros::TimerEvent& event) {
        // This function will be called at rate1_ (100 Hz)
        if (!latest_wrench_.header.stamp.isZero()) {
            // Transform the latest stamped wrench to the desired frame
            geometry_msgs::WrenchStamped transformed_wrench;
            try {
                geometry_msgs::TransformStamped transformStamped, transformation_b_eef;
                transformStamped = tf_buffer_.lookupTransform("tool_link_ee", latest_wrench_.header.frame_id, latest_wrench_.header.stamp, ros::Duration(1.0));
                transformation_b_eef = tf_buffer_.lookupTransform("iiwa_link_0", 
                                                  "tool_link_ee", 
                                                  ros::Time(0));
                tf2::doTransform(latest_wrench_, transformed_wrench, transformStamped);
                wrench_pub_.publish(transformed_wrench);
                eef_pose_.publish(transformation_b_eef);
            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
            }
        }
    }

    void timerCallback2(const ros::TimerEvent& event) {
        // This function will be called at rate2_ (4 Hz)
        if (!latest_wrench_.header.stamp.isZero()) {
            // Transform the latest stamped wrench to the desired frame
            geometry_msgs::WrenchStamped transformed_wrench;
            try {
                geometry_msgs::TransformStamped transformStamped;
                transformStamped = tf_buffer_.lookupTransform("tool_link_ee", latest_wrench_.header.frame_id, latest_wrench_.header.stamp, ros::Duration(1.0));
                tf2::doTransform(latest_wrench_, transformed_wrench, transformStamped);
                wrench_pub_ts.publish(transformed_wrench);
            } catch (tf2::TransformException& ex) {
                ROS_WARN("%s", ex.what());
            }
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "wrench_transform_publisher");
    ros::NodeHandle nh;

    StampedWrenchTransformer node(nh);

    ros::spin();

    return 0;
}
