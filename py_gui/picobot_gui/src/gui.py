from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, Qt, QTime
from PyQt5.QtGui import QPalette, QColor
from ros_interface import RosLaunchProcess, ROSInterface
from styles import *
import rospy
from geometry_msgs.msg import PoseStamped, WrenchStamped, TransformStamped
from std_msgs.msg import Float32
from multiprocessing import Queue
from config import *
import numpy as np
import pyqtgraph as pg
from dynamic_reconfigure.client import Client

class PicobotGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('PicoBot GUI')
        self.setGeometry(100, 100, 400, 800)

        rospy.init_node('picobot_gui', anonymous=True)
        self.uuid = rospy.get_param("/run_id")
        self.pub_home_pose = rospy.Publisher(TOPICS['robot_run'], PoseStamped, queue_size=1)

        self.processes = {}
        self.status_queue = Queue()
        self.force_data = []
        self.plot_run = False
        self.timeout_duration = TIMEOUT['default']
        self.elapsed_time = QTime(0, 0, 0)  # Start at 00:00:00
        self.gui_elapsed_time = QTime(0, 0, 0)  # Start at 00:00:00
        self.dyn_reconf_force = None
        self.force_plot_buffer_duration = 70  # Buffer duration in seconds
        self.max_points = self.force_plot_buffer_duration * 10  # i think it depends on frequency of force topic

        self.init_ui()

        self.timer = QTimer(self)
        self.timer.setInterval(1000)  # this is actually update interval in milliseconds
        self.timer.timeout.connect(self.update_status)
        self.timer.start()

        self.force_controller_timer = QTimer(self)
        self.force_controller_timer.timeout.connect(self.on_force_controller_timeout)

    def init_ui(self):
        # Main Layout
        self.main_layout = QVBoxLayout()

        # PICOBOT STATUS PANEL
        self.status_panel = QWidget(self)
        self.status_panel.setStyleSheet("background-color: white;")

        # self.main_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))
        
        self.label_layout = QHBoxLayout()

        self.fixed_status_label = QLabel("PicoBot Status:  ", self)
        self.fixed_status_label.setStyleSheet("font-size: 30px;")
        # self.fixed_status_label.setAlignment(Qt.AlignCenter)
        self.label_layout.addWidget(self.fixed_status_label)

        self.status_label = QLabel("GUI Running", self)
        self.status_label.setStyleSheet("font-size: 30px;")
        # self.status_label.setAlignment(Qt.AlignCenter)
        self.label_layout.addWidget(self.status_label)

        self.status_panel.setLayout(self.label_layout)
        self.main_layout.addWidget(self.status_panel)

        self.main_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # HOME POSITION BUTTON
        self.home_panel = QWidget(self)
        self.home_layout = QHBoxLayout()

        self.main_clock_label_layout = QVBoxLayout()

        self.main_clock_label_name = QLabel("GUI Elapsed Time", self)
        self.main_clock_label_name.setStyleSheet("font-size: 30px;")
        set_label_color(self.main_clock_label_name, 'green')
        self.main_clock_label_layout.addWidget(self.main_clock_label_name)

        self.main_clock_label = QLabel("00:00:00", self)
        self.main_clock_label.setStyleSheet("font-size: 30px;")
        self.main_clock_label_layout.addWidget(self.main_clock_label)

        self.home_layout.addLayout(self.main_clock_label_layout)

        self.button_go_home = QPushButton('Home Position', self)
        self.button_go_home.setFixedSize(200, 100)
        self.button_go_home.setStyleSheet("font-size: 25px;")
        self.button_go_home.clicked.connect(self.go_home)
        self.home_layout.addWidget(self.button_go_home)

        self.home_panel.setLayout(self.home_layout)
        self.main_layout.addWidget(self.home_panel)

        # Spacer
        self.main_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # CONTROL PANEL: ALL CHECKBOXEs 
        self.control_panel = QWidget(self)
        self.control_panel.setStyleSheet("background-color: white;")

        self.chekbox_layout = QGridLayout()

        self.switch_robot_motion = self.create_checkbox("Robot Motion", self.toggle_robot_motion)
        self.chekbox_layout.addWidget(self.switch_robot_motion, 0, 0)

        self.detection_pose_switch = self.create_checkbox("Detection", self.toggle_detection)
        self.chekbox_layout.addWidget(self.detection_pose_switch, 0, 1)

        self.switch_send_pose = self.create_checkbox("Send Pose", self.toggle_send_pose)
        self.chekbox_layout.addWidget(self.switch_send_pose, 0, 2)

        self.switch_compute_wrench = self.create_checkbox("Get Force", self.toggle_wrench)
        self.chekbox_layout.addWidget(self.switch_compute_wrench, 1, 0)

        self.switch_force_controller = self.create_checkbox("Force Controller", self.toggle_force_controller)
        self.chekbox_layout.addWidget(self.switch_force_controller, 1, 1)

        # self.switch_force_ortn_controller = self.create_checkbox("Orient/Force Cont.", self.toggle_force_oreint_controller)
        # self.chekbox_layout.addWidget(self.switch_force_ortn_controller, 2, 1)

        self.switch_plot_force = self.create_checkbox("Plot Force", self.toggle_plot_force)
        self.chekbox_layout.addWidget(self.switch_plot_force, 1, 2)

        self.bias_force_button = QPushButton('Bias Force', self)
        self.bias_force_button.setFixedSize(150, 50)
        self.bias_force_button.setStyleSheet("font-size: 25px;")
        self.bias_force_button.clicked.connect(self.bias_force_b)
        self.chekbox_layout.addWidget(self.bias_force_button,2,0)

        self.switch_record_THz = self.create_checkbox("THz Record On", self.toggle_record_THz)
        self.chekbox_layout.addWidget(self.switch_record_THz, 2, 2)

        self.control_panel.setLayout(self.chekbox_layout)
        self.main_layout.addWidget(self.control_panel)

        self.main_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        # MESSAGE Lable panel
        self.current_pose_label = QLabel("Current Pose: N/A", self)
        self.current_pose_label.setStyleSheet("font-size: 25px;")
        self.current_pose_label.setAlignment(Qt.AlignCenter)
        # self.main_layout.addWidget(self.current_pose_label)

        self.force_value_label = QLabel("Current Force: N/A", self)
        self.force_value_label.setStyleSheet("font-size: 25px;")
        self.force_value_label.setAlignment(Qt.AlignCenter)
        self.main_layout.addWidget(self.force_value_label)

        # BAG PANEL
        self.bag_panel = QWidget(self)
        self.bag_layout = QGridLayout()

        self.record_bag_button = self.create_checkbox('Record Force/Pose', self.record_bag_b)
        self.bag_layout.addWidget(self.record_bag_button, 0,1)

        self.read_bag_button = QPushButton('Read Bag', self)
        self.read_bag_button.setFixedSize(150, 50)
        self.read_bag_button.setStyleSheet("font-size: 25px;")
        self.read_bag_button.clicked.connect(self.read_bag_b)
        self.bag_layout.addWidget(self.read_bag_button,0,2)

        # self.main_layout.addSpacerItem(QSpacerItem(20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding))

        self.bag_panel.setLayout(self.bag_layout)
        self.main_layout.addWidget(self.bag_panel)

        # FILE NAME PANEL
        self.file_name_panel = QWidget(self)
        self.file_name_layout = QHBoxLayout()
        self.file_name_panel.setStyleSheet("background-color: white;")

        self.file_name_label = QLabel("Files Name:", self)
        self.file_name_label.setStyleSheet("font-size: 20px;")
        self.file_name_layout.addWidget(self.file_name_label)

        self.file_name_input = QLineEdit(self)
        self.file_name_input.setStyleSheet("font-size: 20px;")
        self.file_name_input.setPlaceholderText("Enter_File_Name_with (no_spaces)")
        # self.file_name_input.setFixedWidth(400)
        self.file_name_layout.addWidget(self.file_name_input)

        self.file_name_panel.setLayout(self.file_name_layout)
        self.main_layout.addWidget(self.file_name_panel)

        #FORCE CONTROLLER/TIMER PANEL
        self.force_info_panel = QWidget(self)
        self.force_info_layout = QHBoxLayout()

        self.force_timeout_label = QLabel("Force Controller Timeout:", self)
        self.force_timeout_label.setStyleSheet("font-size: 20px;")
        self.force_info_layout.addWidget(self.force_timeout_label)

        self.force_timeout_spinbox = QSpinBox(self)
        self.force_timeout_spinbox.setStyleSheet("font-size: 20px;")
        self.force_timeout_spinbox.setRange(1, TIMEOUT['max'])  # Set range for timeout in seconds
        self.force_timeout_spinbox.setValue(TIMEOUT['default'])  # Default value
        self.force_timeout_spinbox.setSuffix(' s')
        self.force_info_layout.addWidget(self.force_timeout_spinbox)

        self.dyn_reconf_force_spinbox_label = QLabel("Desired Force:", self)
        self.dyn_reconf_force_spinbox_label.setStyleSheet("font-size: 20px;")
        self.force_info_layout.addWidget(self.dyn_reconf_force_spinbox_label)

        self.dyn_reconf_force_spinbox = QSpinBox(self)
        self.dyn_reconf_force_spinbox.setStyleSheet("font-size: 20px;")
        self.dyn_reconf_force_spinbox.setRange(-2, 10)  # Set range for timeout in seconds
        self.dyn_reconf_force_spinbox.setValue(3)  # Default value
        self.dyn_reconf_force_spinbox.setSuffix(' N')
        self.force_info_layout.addWidget(self.dyn_reconf_force_spinbox)

        self.force_info_panel.setLayout(self.force_info_layout)
        self.main_layout.addWidget(self.force_info_panel)

         # TIMER PANEL
        self.timer_panel = QWidget(self)
        self.timer_layout = QHBoxLayout()
        self.timer_panel.setStyleSheet("background-color: white;")

        self.progress_bar = QProgressBar(self)
        self.progress_bar.setStyleSheet("font-size: 20px;")
        self.progress_bar.setRange(0, TIMEOUT['default'])  # 60 seconds
        self.progress_bar.setValue(0)
        self.progress_bar.setTextVisible(True)
        self.progress_bar.setFixedWidth(400)
        self.timer_layout.addWidget(self.progress_bar)

        self.clock_label = QLabel("00:00:00", self)
        self.clock_label.setStyleSheet("font-size: 40px;")
        set_label_color(self.clock_label, 'red')
        self.timer_layout.addWidget(self.clock_label)

        self.timer_panel.setLayout(self.timer_layout)
        self.main_layout.addWidget(self.timer_panel)

        # PLOT PANEL
        self.force_plot = pg.PlotWidget(self)
        self.force_plot.setBackground('w')  # Set background to white
        self.force_plot.setMinimumSize(400, 200)  # Set minimum size
        self.force_plot.setMaximumSize(800, 800)  # Set maximum size
        self.force_plot.setTitle("Force Value Over Time")
        self.force_plot.setLabel('left', 'Force', 'N')
        self.force_plot.setLabel('bottom', 'Time', 's')
        self.force_plot_curve = self.force_plot.plot(pen=pg.mkPen(color='r', width=2))
        self.main_layout.addWidget(self.force_plot)

        # SET MAIN LAYOUT
        container = QWidget()
        container.setLayout(self.main_layout)
        self.setCentralWidget(container)

############################### Functions ###############################
    def create_checkbox(self, text, method):
        checkbox = QCheckBox(text, self)
        checkbox.stateChanged.connect(method)
        set_checkbox_style(checkbox, 'black')
        return checkbox

    def go_home(self):
        if 'force_controller' in self.processes and self.processes['force_controller'].is_alive():
            self.status_label.setText("Cannot move: Force controller is ON")
            set_label_color(self.status_label, 'red')
            return
        if 'robot_motion' not in self.processes or not self.processes['robot_motion'].is_alive():
            self.status_label.setText("Cannot move: Robot Motion is OFF")
            set_label_color(self.status_label, 'red')
            return

        self.status_label.setText("Home Pose: Sending.../Reached")
        self.reset_main_label()
        home_pose = get_home_pose()
        self.pub_home_pose.publish(home_pose)

    def toggle_robot_motion(self, state):
        self.toggle_process(state, 'robot_motion', LAUNCH_FILES['robot_motion'])

    def toggle_detection(self, state):
        self.toggle_process(state, 'detection', LAUNCH_FILES['detection'])

    def toggle_wrench(self, state):
        self.toggle_process(state, 'wrench', LAUNCH_FILES['wrench'])
        if state == 2:
            self.force_subscriber = rospy.Subscriber(TOPICS['force_topic'], WrenchStamped, self.update_force_value)
            self.current_pose_subscriber = rospy.Subscriber(TOPICS['current_pose'], TransformStamped, self.update_current_pose_value)
        else:
            if hasattr(self, 'force_subscriber'):
                self.force_subscriber.unregister()
                self.current_pose_subscriber.unregister()
                self.force_value_label.setText("Force in Z:   N/A")
                self.current_pose_label.setText("Current Pose: N/A")

    def toggle_force_controller(self, state):
        if 'wrench' not in self.processes or not self.processes['wrench'].is_alive():
            self.status_label.setText("Cannot move: Get Force is OFF")
            set_label_color(self.status_label, 'red')
            return
        # if 'contact_force_controller' in self.processes and self.processes['contact_force_controller'].is_alive():
        #     self.status_label.setText("Cannot apply: Oreint/Force controller is ON")
        #     set_label_color(self.status_label, 'red')
        #     return
            
        if state == 2:
            self.toggle_process(state, 'force_controller', LAUNCH_FILES['force_controller'])
            self.dyn_reconf_force_client = Client('force_controller', timeout=5) # node name
            self.dyn_reconf_force = self.dyn_reconf_force_spinbox.value()
            self.dyn_reconf_force_client.update_configuration({'desired_force': self.dyn_reconf_force})

            self.timeout_duration = self.force_timeout_spinbox.value()
            self.progress_bar.setRange(0, self.timeout_duration)
            # self.force_controller_timer.start(self.timeout_duration*1000)
            self.force_controller_timer.start(1000)  # Start a timer to update progress bar
            if 'record_thz' not in self.processes:
                self.status_label.setText("Force Controller running")
                set_label_color(self.status_label, 'green')
            else:
                self.status_label.setText("Force Cont. ON + THz recording")
                set_label_color(self.status_label, 'green')
                
        else:
            self.force_controller_timer.stop()
            self.elapsed_time = QTime(0, 0, 0)
            self.progress_bar.setValue(0)
            self.reset_main_label()
            if 'force_controller' in self.processes:
                self.processes['force_controller'].terminate()
                self.processes['force_controller'].join()
                del self.processes['force_controller']

    # def toggle_force_oreint_controller(self, state):
    #     if 'wrench' not in self.processes or not self.processes['wrench'].is_alive():
    #         self.status_label.setText("Cannot move: Get Force is OFF")
    #         set_label_color(self.status_label, 'red')
    #         return
    #     if 'force_controller' in self.processes and self.processes['force_controller'].is_alive():
    #         self.status_label.setText("Cannot apply: Force controller is ON")
    #         set_label_color(self.status_label, 'red')
    #         return
            
    #     if state == 2:
    #         self.toggle_process(state, 'contact_force_controller', LAUNCH_FILES['contact_force_controller'])
    #         self.dyn_reconf_force_client = Client('contact_force_controller', timeout=5) # node name
    #         self.dyn_reconf_force = self.dyn_reconf_force_spinbox.value()
    #         self.dyn_reconf_force_client.update_configuration({'desired_force': self.dyn_reconf_force})

    #         # self.timeout_duration = self.force_timeout_spinbox.value()
    #         # self.progress_bar.setRange(0, self.timeout_duration)
    #         # self.force_controller_timer.start(self.timeout_duration*1000)
    #         # self.force_controller_timer.start(1000)  # Start a timer to update progress bar
    #         if 'record_thz' not in self.processes:
    #             self.status_label.setText("Force Controller running")
    #             set_label_color(self.status_label, 'green')
    #         else:
    #             self.status_label.setText("Force Cont. ON + THz recording")
    #             set_label_color(self.status_label, 'green')
                
    #     else:
    #         # self.force_controller_timer.stop()
    #         # self.elapsed_time = QTime(0, 0, 0)
    #         # self.progress_bar.setValue(0)
    #         self.reset_main_label()
    #         if 'contact_force_controller' in self.processes:
    #             self.processes['contact_force_controller'].terminate()
    #             self.processes['contact_force_controller'].join()
    #             del self.processes['contact_force_controller']

    def toggle_record_THz(self, state):
        if state == 2:
            time = self.force_timeout_spinbox.value()
            file_name = self.file_name_input.text()
            LAUNCH_FILES['record_thz'][1] = "file:=" + file_name
            LAUNCH_FILES['record_thz'][2] = "time:=" + str(time)
            # print(LAUNCH_FILES['record_thz'][1])
            self.status_label.setText("Recording THz")
            set_label_color(self.status_label, 'green')
            self.record_bag_b(2) # automatically starts recording force and pose.
        else:
            self.record_bag_b(0)
        self.toggle_process(state, 'record_thz', LAUNCH_FILES['record_thz'])

    def toggle_send_pose(self, state):
        if 'robot_motion' not in self.processes or not self.processes['robot_motion'].is_alive():
            self.status_label.setText("Cannot move: Robot Motion is OFF")
            set_label_color(self.status_label, 'red')
            return
        self.toggle_process(state, 'send_pose', LAUNCH_FILES['send_pose'])
        # self.switch_send_pose.setChecked(False)

    def toggle_plot_force(self, state):
        if state == 2:
            self.plot_run=True
        else:
            self.plot_run=False

    def record_bag_b(self, state):
        if state == 2:
            file_name = self.file_name_input.text()
            LAUNCH_FILES['record_bag'][1] = "file:=" + file_name
        self.toggle_process(state, 'record_bag', LAUNCH_FILES['record_bag'])

    def read_bag_b(self):
        file_name = self.file_name_input.text()
        LAUNCH_FILES['read_bag'][1] = "file:=" + file_name
        self.toggle_process(2, 'read_bag', LAUNCH_FILES['read_bag'])

    def bias_force_b(self):
        call_bias_wrench_service(True)

    def update_force_value(self, msg):
        if msg is None:
            self.status_label.setText("Force N/A | Check Robot Status")
            set_label_color(self.status_label, 'red')
            return
        self.force_value_label.setText(f"Force in Z:   {np.round(msg.wrench.force.z, 3)} N")
        if self.plot_run:
            self.force_data.append(msg.wrench.force.z)
            # Keep only the most recent max_points in the buffer
            if len(self.force_data) > self.max_points:
                self.force_data = self.force_data[-self.max_points:]
            self.update_force_plot()
        else:
            self.force_data = []
            self.update_force_plot()
            pass

    def update_force_plot(self):
        self.force_plot_curve.setData(self.force_data)

    def update_current_pose_value(self, msg):
        self.current_pose_label.setText(f"Current Pose: X: {np.round(msg.transform.translation.x,3)},  Y: {np.round(msg.transform.translation.y,3)},  Z: {np.round(msg.transform.translation.z,3)}")

    def on_force_controller_timeout(self):
        current_value = self.progress_bar.value()
        if current_value < self.timeout_duration:
            self.progress_bar.setValue(current_value + 1)
            self.elapsed_time = self.elapsed_time.addSecs(1)
            self.clock_label.setText(self.elapsed_time.toString("hh:mm:ss"))
        else:
            self.force_controller_timer.stop()
            # self.toggle_record_THz(0)  # Turn off the record THz
            # self.switch_record_THz.setChecked(False)
            # self.toggle_force_controller(0)  # Turn off the force controller
            # self.switch_force_controller.setChecked(False)
            # self.go_home()  # Send the robot to home pose

    def update_status(self):
        self.gui_elapsed_time = self.gui_elapsed_time.addSecs(1)
        self.main_clock_label.setText(self.gui_elapsed_time.toString("hh:mm:ss"))
        set_label_color(self.main_clock_label, 'green')
        if self.gui_elapsed_time > QTime(0, 45, 0):
            self.status_label.setText("REBOOT ROBOT!!")
            set_label_color(self.status_label, 'red')
            set_label_color(self.main_clock_label, 'red')
        while not self.status_queue.empty():
            try:
                launch_file, status = self.status_queue.get_nowait()
                print(f"Launch file {launch_file} has {status}")
            except Exception as e:
                print(f"Error retrieving status: {e}")

    def reset_main_label(self):
        # self.status_label.setText("Ready")
        set_label_color(self.status_label, 'black')

    def toggle_process(self, state, name, launch_file):
        if state == 2:
            process = RosLaunchProcess(self.uuid, PACKAGE_NAME, launch_file, self.status_queue)
            process.start()
            self.processes[name] = process
        else:
            if name in self.processes:
                self.processes[name].terminate()
                self.processes[name].join()
                del self.processes[name]