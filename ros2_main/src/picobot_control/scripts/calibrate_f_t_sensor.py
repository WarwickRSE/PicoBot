import rospy
from geometry_msgs.msg import WrenchStamped, TransformStamped
import tf2_ros
import threading

class CalibrateFTSensor:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('calibrate_f_t_sensor', anonymous=True)
        self.rate =rospy.Rate(100)

        # Initialize TF2 buffer and listener
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.transformStamped = TransformStamped()
        self.wrench_data = WrenchStamped()

         # List to store recorded data
        self.recorded_g_transforms = []
        self.recorded_forces = []

        # Subscriber for force-torque sensor data
        rospy.Subscriber('/netft_data', WrenchStamped, self.callback_ft)

        # Thread flag
        self.recording_flag = False
        self.stop_recording = False  # Flag to stop recording when user decides

        # Start a separate thread to handle user input
        input_thread = threading.Thread(target=self.wait_for_enter)
        input_thread.start()

    def get_transform_info(self):
        try:
            self.transformStamped = self.tfBuffer.lookup_transform('iiwa_link_0', 'sensor_link', rospy.Time(0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Error in lookup transform: {e}")

    def callback_ft(self, msg):
        # Get the latest transform from the sensor to the base frame
        self.wrench_data = msg

    def wait_for_enter(self):
        # Handle user input asynchronously
        while not rospy.is_shutdown() and not self.stop_recording:
            user_input = input("Press Enter to record data, or type 'done' to finish: ")
            if user_input.lower() == "done":
                self.stop_recording = True
                rospy.loginfo("Recording stopped. Processing data...")
            else:
                self.recording_flag = True

    def record_data(self):
        # Record the current wrench and transform data
        self.get_transform_info()
        rospy.loginfo("Recording data...")
        # self.recorded_transforms.append(self.transformStamped)
        # self.recorded_wrenches.append(self.wrench_data)
        rospy.loginfo(f"Transform: {self.transformStamped}")
        rospy.loginfo(f"Wrench: {self.wrench_data}")
        rospy.loginfo("Data recorded.\n")

    def run(self):
        # Main loop
        while not rospy.is_shutdown():
            # If the recording flag is set, record data
            if self.recording_flag:
                self.record_data()
                self.recording_flag = False

            self.rate.sleep()

if __name__ == '__main__':
    calibrator = CalibrateFTSensor()
    calibrator.run()
    