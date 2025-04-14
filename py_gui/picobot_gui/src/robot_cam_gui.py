import sys
import rospy
import roslaunch
from std_msgs.msg import String
from PyQt5.QtWidgets import QApplication,QMainWindow,QWidget,QLabel,QVBoxLayout,QHBoxLayout,QPushButton,QFileDialog, QGridLayout, QFormLayout
from PyQt5.QtCore import QTimer
import os

class RobotCamGui(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('RobotCam GUI')
        self.setGeometry(100, 100, 600, 300)

        self.current_file_path= os.path.dirname(os.path.abspath(__file__))
        self.directory_path = os.path.dirname(self.current_file_path)
        self.src_path = os.path.dirname(self.directory_path)

        self.robot_sim = None
        self.robot_real = None
        self.camera = None

        # rospy.init_node('robot_cam_gui', anonymous=True)

        # GUI elements
        self.main_layout = QVBoxLayout()
        #status label
        self.status_label = QLabel("Robot Status: None",self)
        self.status_label.setStyleSheet("font-size: 24px;")
        self.main_layout.addWidget(self.status_label)

        # create a layout for buttons
        self.button_layout = QGridLayout()

        # self.button_robot_launch_sim = QPushButton('Launch Simulation',self)
        # self.button_robot_launch_sim.setFixedSize(200,80)
        # self.button_robot_launch_sim.setStyleSheet("font-size: 20px;")
        # self.button_robot_launch_sim.clicked.connect(self.robot_launch_sim)
        # self.button_layout.addWidget(self.button_robot_launch_sim,1,0)

        self.button_robot_launch_real = QPushButton('Launch KUKA',self)
        self.button_robot_launch_real.setFixedSize(200,80)
        self.button_robot_launch_real.setStyleSheet("font-size: 20px;")
        self.button_robot_launch_real.clicked.connect(self.launch_real)
        self.button_layout.addWidget(self.button_robot_launch_real,3,0)

        self.button_robot_stop = QPushButton('Stop Robot',self)
        self.button_robot_stop.setFixedSize(200,80)
        self.button_robot_stop.setStyleSheet("font-size: 20px;")
        self.button_robot_stop.clicked.connect(self.stop_robot)
        self.button_layout.addWidget(self.button_robot_stop,3,1)
       
        self.button_cam_launch = QPushButton('Launch Camera',self)
        self.button_cam_launch.setFixedSize(200,80)
        self.button_cam_launch.setStyleSheet("font-size: 20px;")
        self.button_cam_launch.clicked.connect(self.launch_cam)
        self.button_layout.addWidget(self.button_cam_launch,5,0)

        self.button_cam_stop = QPushButton('Stop Camera',self)
        self.button_cam_stop.setFixedSize(200,80)
        self.button_cam_stop.setStyleSheet("font-size: 20px;")
        self.button_cam_stop.clicked.connect(self.cam_stop)
        self.button_layout.addWidget(self.button_cam_stop,5,1)

        self.main_layout.addLayout(self.button_layout)


        #timer for periodic updates
        self.timer = QTimer(self)
        # self.timer.timeout.connect(self.update_status)
        self.timer.start(100)

        container = QWidget()
        container.setLayout(self.main_layout)
        self.setCentralWidget(container)

    def robot_launch_sim(self):
        print("Launching simulation")
        self.status_label.setText("Robot Status: Launching simulation")
        # global robot_sim
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [self.src_path+ "/iiwa_ros/iiwa_gazebo/launch/iiwa_gazebo.launch",'model:=iiwa14', 'controller:=CartesianImpedance_trajectory_controller']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self.robot_sim = roslaunch.parent.ROSLaunchParent(uuid,roslaunch_file)
        self.robot_sim.start()
        self.set_button_color(self.button_robot_launch_sim, 'green')
        # rospy.loginfo("robot sim started")

    def launch_real(self):
        print("Launching real robot")
        self.status_label.setText("Robot Status: Launching real robot")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [self.src_path+"/iiwa_ros/iiwa_driver/launch/iiwa_bringup.launch",'model:=iiwa14', 'controller:=CartesianImpedance_trajectory_controller']
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self.robot_real = roslaunch.parent.ROSLaunchParent(uuid,roslaunch_file)
        self.robot_real.start()
        self.set_button_color(self.button_robot_launch_real, 'green')
        # rospy.loginfo("robot started")

    def launch_cam(self):
        if self.robot_real or self.robot_sim is not None:
            self.status_label.setText("Robot Status: Launching camera and Robot")
        else:
            self.status_label.setText("Camera launching but robot is not running")
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        cli_args = [self.src_path+"/picobot_control/launch/cam_viz.launch"]
        roslaunch_args = cli_args[1:]
        roslaunch_file = [(roslaunch.rlutil.resolve_launch_arguments(cli_args)[0], roslaunch_args)]
        self.camera = roslaunch.parent.ROSLaunchParent(uuid,roslaunch_file)
        self.camera.start()
        self.set_button_color(self.button_cam_launch, 'green')
        # rospy.loginfo("camera started")

    def stop_robot(self):
        print("Stopping robot")
        # if self.robot_sim is not None:
        #     self.robot_sim.shutdown()
        #     self.set_button_color(self.button_robot_launch_sim, 'red')
            
        if self.robot_real is not None:
            self.robot_real.shutdown()
            self.status_label.setText("Robot Status: Stopping robot")
            self.set_button_color(self.button_robot_launch_real, 'red')
        else:
            self.status_label.setText("Robot not running")

    def cam_stop(self):
        if self.camera is not None:
            self.camera.shutdown()
            self.set_button_color(self.button_cam_launch, 'red')
        else:
            self.status_label.setText("Camera not running")

    def ros_spin(self):
        pass

    def set_button_color(self, button, color):
        button.setStyleSheet("background-color: {}".format(color))

    def update_status(self):
        status = "None"
        self.status_label.setText("Robot Status: {}".format(status))
        self.reset_button_colors()

    def reset_button_colors(self):
        self.set_button_color(self.button_robot_launch_sim, 'red')
        self.set_button_color(self.button_robot_launch_real, 'red')
        

if __name__ == '__main__':
    app = QApplication(sys.argv)
    gui = RobotCamGui()
    gui.show()
    sys.exit(app.exec_())