import roslaunch
from multiprocessing import Process, Queue

class RosLaunchProcess(Process):
    def __init__(self, uuid, name, launch_file, status_queue):
        super().__init__()
        self.uuid = uuid
        self.name = name
        self.launch_file = launch_file
        self.status_queue = status_queue

    def run(self):
        roslaunch_file = roslaunch.rlutil.resolve_launch_arguments([self.launch_file[0]])[0] #  this is incorporate in list in the LAUNCH_FILES
        launch_args = self.launch_file[1:] if len(self.launch_file) > 1 else []
        roslaunch_file = [(roslaunch_file, launch_args)]
        self.launch = roslaunch.parent.ROSLaunchParent(self.uuid, roslaunch_file)
        self.launch.start()
        self.status_queue.put((self.launch_file, 'started'))
        self.launch.spin()
        self.status_queue.put((self.launch_file, 'stopped'))

class ROSInterface:
    def __init__(self):
        self.uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(self.uuid)
        self.processes = {}

    def start_process(self, name, launch_file):
        status_queue = Queue()
        process = RosLaunchProcess(self.uuid, name, launch_file, status_queue)
        process.start()
        self.processes[name] = (process, status_queue)
        return status_queue

    def stop_process(self, name):
        if name in self.processes:
            process, _ = self.processes[name]
            process.terminate()
            process.join()
            del self.processes[name]
