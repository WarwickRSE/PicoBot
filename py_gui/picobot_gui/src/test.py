import sys
import rospy
import roslaunch
from geometry_msgs.msg import PoseStamped, WrenchStamped
from PyQt5.QtWidgets import QApplication, QMainWindow, QWidget, QLabel, QCheckBox, QVBoxLayout, QPushButton, QGridLayout
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.QtGui import QPalette, QColor
from multiprocessing import Process, Queue
import os

class RosLaunchProcess(Process):
    def __init__(self, uuid, package, launch_file, status_queue):
        super().__init__()
        self.uuid = uuid
        self.package = package
        self.launch_file = launch_file
        self.status_queue = status_queue

    def run(self):
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments([self.launch_file])[0])]
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        self.launch.start()
        self.status_queue.put((self.launch_file,'started'))
        self.launch.spin()
        self.status_queue.put((self.launch_file,'stopped'))

class PicobotGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('PicoBot GUI')
        self.setGeometry(100, 100, 600, 600)

        rospy.init_node('picobot_gui', anonymous=True)
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        
        self.pub_home_pose = rospy.Publisher("/cartesian_trajectory_generator/new_goal", PoseStamped, queue_size=1)

        self.processes = {}
        self.status_queue = Queue()

        self.robot_motion_thread = None
        self.markers_thread = None
        self.wrenches_thread = None
        self.force_controller_thread = None
        self.target_pose_thread = None

        self.init_ui()
        
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_status)
        self.timer.start(1000)

    def init_ui(self):
        self.main_layout = QVBoxLayout()
        
        self.status_label = QLabel("PicoBot Status: GUI Running", self)
        self.status_label.setStyleSheet("font-size: 20px;")
        self.main_layout.addWidget(self.status_label)

        self.button_go_home = QPushButton('Home Position', self)
        self.button_go_home.setFixedSize(1000, 30)
        self.button_go_home.setStyleSheet("font-size: 20px;")
        self.button_go_home.clicked.connect(self.go_home)
        self.main_layout.addWidget(self.button_go_home)

        self.button_layout = QGridLayout()

        self.switch_robot_motion = self.create_checkbox("Robot Motion", self.toggle_robot_motion)
        self.button_layout.addWidget(self.switch_robot_motion, 0, 0)

        self.detection_pose_switch = self.create_checkbox("Detection", self.toggle_detection)
        self.button_layout.addWidget(self.detection_pose_switch, 1, 0)

        self.switch_send_pose = self.create_checkbox("Send Pose", self.toggle_send_pose)
        self.button_layout.addWidget(self.switch_send_pose, 2, 0)

        self.switch_compute_wrench = self.create_checkbox("Compute Wrench", self.toggle_wrench)
        self.button_layout.addWidget(self.switch_compute_wrench, 3, 0)

        self.switch_force_controller = self.create_checkbox("Force Controller", self.toggle_force_controller)
        self.button_layout.addWidget(self.switch_force_controller, 4, 0)

        self.force_value_label = QLabel("F_z: N/A", self)
        self.force_value_label.setStyleSheet("font-size: 20px;")
        self.button_layout.addWidget(self.force_value_label,4,1)

        self.main_layout.addLayout(self.button_layout)

        container = QWidget()
        container.setLayout(self.main_layout)
        self.setCentralWidget(container)

    def create_checkbox(self, text, method):
        checkbox = QCheckBox(text, self)
        checkbox.stateChanged.connect(method)
        self.set_checkbox_style(checkbox, 'red')
        return checkbox

    def go_home(self):
        if 'force_controller' in self.processes and self.processes['force_controller'].is_alive():
            self.status_label.setText("Cannot move: Force controller is ON")
            return
        if 'robot_motion' not in self.processes or not self.processes['robot_motion'].is_alive():
            self.status_label.setText("Cannot move: Robot Motion is OFF")
            return

        self.status_label.setText("Sending home pose")
        home_pose = PoseStamped()
        home_pose.header.frame_id = "world"
        home_pose.header.stamp = rospy.Time.now()
        home_pose.pose.position.x = -0.6
        home_pose.pose.position.y = 0
        home_pose.pose.position.z = 0.3
        home_pose.pose.orientation.x = 0.7
        home_pose.pose.orientation.y = 0.7
        home_pose.pose.orientation.z = 0
        home_pose.pose.orientation.w = 0
        self.pub_home_pose.publish(home_pose)

    def toggle_robot_motion(self, state):
        self.toggle_process(state, 'robot_motion', '/home/terabotics/stuff_ws/src/tera_iiwa_ros/launch/init_robot_motion.launch')

    def toggle_detection(self, state):
        self.toggle_process(state, 'detection', '/home/terabotics/stuff_ws/src/tera_iiwa_ros/launch/one_in_all.launch')

    def toggle_wrench(self, state):
        self.toggle_process(state, 'wrench', '/home/terabotics/stuff_ws/src/tera_iiwa_ros/launch/get_wrench_sim.launch')
        if state == 2:
            self.force_subscriber = rospy.Subscriber('/cartesian_wrench_tool', WrenchStamped, self.update_force_value)
        else:
            if hasattr(self, 'force_subscriber'):
                self.force_subscriber.unregister()
                self.force_value_label.setText(" F_z: N/A")
    def toggle_force_controller(self, state):
        self.toggle_process(state, 'force_controller', '/home/terabotics/stuff_ws/src/tera_iiwa_ros/launch/force_controller.launch')

    def toggle_send_pose(self, state):
        self.toggle_process(state, 'send_pose', '/home/terabotics/stuff_ws/src/tera_iiwa_ros/launch/send_target_pose.launch')

    def toggle_process(self, state, name, launch_file):
        if state == 2:
            process = RosLaunchProcess(self.uuid, 'tera_iiwa_ros', launch_file, self.status_queue)
            process.start()
            self.processes[name] = process
        else:
            if name in self.processes:
                self.processes[name].terminate()
                self.processes[name].join()
                del self.processes[name]

    def set_checkbox_style(self, checkbox, color):
        palette = checkbox.palette()
        # palette.setColor(QPalette.WindowText, QColor(color))
        checkbox.setPalette(palette)
        checkbox.setStyleSheet("QCheckBox { font-size: 20px; } QCheckBox::indicator { width: 40px; height: 40px; }")

    def update_force_value(self, msg):
        self.force_value_label.setText(f"F_z: {msg.wrench.force.z} N")
    def update_status(self):
        while not self.status_queue.empty():
            try:
                launch_file,status = self.status_queue.get_nowait()
                # Update status label or other status-related elements here
                print(f"Launch file {launch_file} has {status}")
            except Exception as e:
                print(f"Error retrieving status: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = PicobotGui()
    gui.show()
    sys.exit(app.exec_())
