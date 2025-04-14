import rospy
import tf.transformations
import tf2_ros
import numpy as np
from geometry_msgs.msg import WrenchStamped
from scipy.signal import butter, filtfilt
from picobot_control.srv import SensorBias, SensorBiasResponse
import copy

class GravityCompensationNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('grav_comp', anonymous=True)
        self.rate =rospy.Rate(100)

        #butterworth filter parameters
        self.cutoff_frequency = 0.5  # Hz
        self.sampling_rate = 100.0  # Hz
        self.order = 2
        #filter coefficients
        self.b, self.a =self.butterworth_filter(self.cutoff_frequency, self.sampling_rate, self.order)

        self.window_size = 30

        # Initialize TF2 buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        # Publisher for compensated wrench
        self.pub = rospy.Publisher('/cartesian_wrench_tool', WrenchStamped, queue_size=1)
        self.pub_check = rospy.Publisher('/cartesian_wrench_tool_unfiltered', WrenchStamped, queue_size=1)
        self.pub_biased = rospy.Publisher('/cartesian_wrench_tool_biased', WrenchStamped, queue_size=1)
        # Subscriber for force-torque sensor data
        rospy.Subscriber('/netft_data', WrenchStamped, self.callback_)

        # Precompute constants
        # v3
        # self.F_bias = np.array([-3.15, 0.1867, 0.8952])
        # self.T_bias = np.array([0.0918, -0.0558, -0.0525])
        # mass = 3.5245
        # com = np.array([0.0178, -0.0008, 0.0798])

        # v4 
        self.F_bias = np.array([-3.474, -0.697, -0.611])
        self.T_bias = np.array([0.00731, 0.08714, -0.0264])
        mass = 3.4431
        com = np.array([0.01507, -0.00598, 0.07139])
        
        self.Fmg = np.array([0, 0, -mass * 9.81065])  # Adjust force due to gravity
        self.zero_vec = np.zeros(3)
        # self.P_s_g = np.array([
        #     [0, -0.079015, 0.00316], 
        #     [0.079015, 0.0, -0.00918],
        #     [-0.00316, 0.00918, 0.0]
        # ])
        self.P_s_g = np.array([
            [0, -com[2], com[1]], 
            [com[2], 0.0, -com[0]],
            [-com[1], com[0], 0.0]
        ])
        # Form wrench vector
        self.wrench_vector = np.hstack((self.Fmg, self.zero_vec)).reshape(6, 1)

        self.force_data = []
        self.torque_data = []
        self.out = WrenchStamped()
        self.out_check = WrenchStamped()
        self.biased_wrench = WrenchStamped()
        self.wrench_for_bias = WrenchStamped()
        self.set_bias = False
        self.first_time_biased = False

        self.service = rospy.Service('/set_sensor_bias', SensorBias, self.service_callback)

    def butterworth_filter(self, cutoff_frequency, sampling_rate, order):
        nyquist_frequency = 0.5 * sampling_rate
        normalized_cutoff_frequency = cutoff_frequency / nyquist_frequency
        b, a = butter(order, normalized_cutoff_frequency, btype='low', analog=False)
        return b, a
    
    def apply_filter(self, data):
        if len(data) < self.order +1: #Ensure we have enough data
            return data
        #Apply butterworth filter
        return filtfilt(self.b, self.a, data)
    
    def get_gravity_wrench(self):
        try:
            # Lookup the transform from the base link to the sensor link
            transformStamped = self.tfBuffer.lookup_transform('iiwa_link_0', 'sensor_link', rospy.Time(0))

            # Extract translation (position)
            translation = transformStamped.transform.translation
            x, y, z = translation.x, translation.y, translation.z

            # Extract orientation (quaternion)
            rotation = transformStamped.transform.rotation
            qx, qy, qz, qw = rotation.x, rotation.y, rotation.z, rotation.w

            # Convert quaternion to rotation matrix
            q = [qx, qy, qz, qw]
            R = tf.transformations.quaternion_matrix(q)[:3, :3].T # with respect to sensor frame I guess, thats why transposed
            # R = R_ @ np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
            # Construct the spatial transformation matrix F_s_g
            F_s_g = np.block([
                [R, 0*np.eye(3)],           # Upper half (rotation and identity)
                [(self.P_s_g @ R), R]  # Lower half (cross product and rotation)
            ])

            # Compute the compensated wrench
            result = F_s_g @ self.wrench_vector

            # print(result) 
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error in lookup transform: {e}")
        # print(result)
        return result


    def callback_(self, msg): 

        # Append new force and torque data to buffers
        self.force_data.append([msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z])
        self.torque_data.append([msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z])

        if len(self.force_data) > self.sampling_rate:
            self.force_data.pop(0)
            self.torque_data.pop(0)
        else:
            return

        # if len(self.force_data) >= self.order + 1:
       
        # Get the latest filtered data
        filtered_force = self.apply_filter(np.array(self.force_data).T).T[-1]
        filtered_torque = self.apply_filter(np.array(self.torque_data).T).T[-1]
        # filtered_force = np.mean(self.force_data[-self.window_size:], axis=0)  # Moving average
        # filtered_torque = np.mean(self.torque_data[-self.window_size:], axis=0)

        result = self.get_gravity_wrench()

        self.out.header = msg.header
        self.out.wrench.force.x     = filtered_force[0]   - result[0] - self.F_bias[0] 
        self.out.wrench.force.y     = filtered_force[1]   - result[1] - self.F_bias[1]
        self.out.wrench.force.z     = filtered_force[2]   - result[2] - self.F_bias[2]
        self.out.wrench.torque.x    = filtered_torque[0]  - result[3] - self.T_bias[0]
        self.out.wrench.torque.y    = filtered_torque[1]  - result[4] - self.T_bias[1]
        self.out.wrench.torque.z    = filtered_torque[2]  - result[5] - self.T_bias[2]

        if self.first_time_biased:
            self.set_bias_value()

        if self.set_bias:
            self.biased_wrench.header = msg.header
            self.biased_wrench.wrench.force.x     = self.out.wrench.force.x - self.wrench_for_bias.wrench.force.x
            self.biased_wrench.wrench.force.y     = self.out.wrench.force.y - self.wrench_for_bias.wrench.force.y
            self.biased_wrench.wrench.force.z     = self.out.wrench.force.z - self.wrench_for_bias.wrench.force.z
            self.biased_wrench.wrench.torque.x    = self.out.wrench.torque.x   - self.wrench_for_bias.wrench.torque.x
            self.biased_wrench.wrench.torque.y    = self.out.wrench.torque.y   - self.wrench_for_bias.wrench.torque.y
            self.biased_wrench.wrench.torque.z    = self.out.wrench.torque.z   - self.wrench_for_bias.wrench.torque.z

        self.out_check.header = msg.header
        self.out_check.wrench.force.x     = msg.wrench.force.x   - result[0]
        self.out_check.wrench.force.y     = msg.wrench.force.y   - result[1]
        self.out_check.wrench.force.z     = msg.wrench.force.z   - result[2]
        self.out_check.wrench.torque.x    = msg.wrench.torque.x  - result[3]
        self.out_check.wrench.torque.y    = msg.wrench.torque.y  - result[4]
        self.out_check.wrench.torque.z    = msg.wrench.torque.z  - result[5]

    def set_bias_value(self):
        self.wrench_for_bias = copy.deepcopy(self.out)
        self.first_time_biased=False
        print("Bias set successfully", self.wrench_for_bias)
        
    def publish(self):
        while not rospy.is_shutdown():
            self.pub.publish(self.out)
            self.pub_check.publish(self.out_check)
            if self.set_bias:
                self.pub_biased.publish(self.biased_wrench)
            self.rate.sleep()

    def service_callback(self, req):
        if req.ft_sensor_bias:
            self.set_bias = True
            self.first_time_biased = True
            return SensorBiasResponse(success=True)
        else:
            self.set_bias = False
            return SensorBiasResponse(success=False)

if __name__ == '__main__':
    node = GravityCompensationNode()
    node.publish()
