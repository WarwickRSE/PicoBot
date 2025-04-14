import os
from geometry_msgs.msg import PoseStamped
import rclpy
from picobot_control_interfaces.srv import *

current_file_path= os.path.dirname(os.path.abspath(__file__))
directory_path = os.path.dirname(current_file_path)
src_path = os.path.dirname(directory_path)

TIMEOUT = {'default': 500,
           'max': 1860}

# ROS Topics
TOPICS = {
    'robot_run': '/cartesian_trajectory_generator/new_goal',
    'force_topic': '/cartesian_wrench_tool_biased',
    'current_pose': '/tool_link_ee_pose'
}
SERVICE = {
    'set_sensor_bias': '/set_sensor_bias'
}


# Home Pose
def get_home_pose():
    home_pose = PoseStamped()
    home_pose.header.frame_id = "world"
    home_pose.header.stamp = rospy.Time.now()
    home_pose.pose.position.x = 0.525
    home_pose.pose.position.y = 0
    home_pose.pose.position.z = 0.35
    home_pose.pose.orientation.x = 0.7
    home_pose.pose.orientation.y = 0.7
    home_pose.pose.orientation.z = 0
    home_pose.pose.orientation.w = 0
    return home_pose


def call_bias_wrench_service(bias):
    rospy.wait_for_service(SERVICE['set_sensor_bias'],timeout=3)
    try:
        set_bias = rospy.ServiceProxy('/set_sensor_bias', SensorBias)
        req = SensorBiasRequest(ft_sensor_bias=bias)
        response = set_bias(req)
        if response.success:
            rospy.loginfo("Bias set successfully")
        else:
            rospy.loginfo("Failed to set bias")
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)



