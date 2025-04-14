#!/usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped, TransformStamped
import socket, struct, sys, time, os
import message_filters

class THzROS:
    def __init__(self, ip, port,base_file_name, total_time):
        # THz system communication and settings
        self.ip_addr = ip
        self.port = port
        self.total_time = int(total_time) # coz input is of type str
        # print(type(self.total_time))
        # self.start_time = -250 #-340
        # self.end_time = -200 #-280
        return_value = self.submit("STOP","?", 8)#Starting/Stopping Spectrometer if scanning
        return_value = self.submit("GETSTATUS","i",32)#getting the status
        return_value = self.submit("SETSTARTVALUE -250", "?", 8)#setting the start of the scan
        return_value = self.submit("SETENDVALUE -200", "?", 8)#setting the end of the scan
        return_value = self.submit("START","?", 8)#StartingSpectrometer if scanning
        print("Connected to TeraSMART scan control system")
        time.sleep(2)

        self.numpoints = self.submit("GETNUMBEROFPOINTS", "i", 32) #number of points of the pulse
        print("numpoints: ",  self.numpoints[0])
        time_axis_bytes = 8 * self.numpoints[0]
        self.time_axis = self.submit("GETTIMEAXIS", "d", time_axis_bytes) #array with the time data
         
        rospy.init_node('thz_ros', anonymous=True)
        # self.data_subscriber = rospy.Subscriber('/cartesian_wrench_tool_ts', WrenchStamped, self.callback_wrench)
        self.ee_pose_message_filter = message_filters.Subscriber('/tool_link_ee_pose', TransformStamped)
        self.wrench_tool_message_filter = message_filters.Subscriber('/cartesian_wrench_tool_ts', WrenchStamped)
        self.ts = message_filters.ApproximateTimeSynchronizer([self.ee_pose_message_filter, self.wrench_tool_message_filter], 10, 0.1, allow_headerless=True)
        self.ts.registerCallback(self.record_callback)
        self.base_file_name = base_file_name
        home = os.path.expanduser("~")
        self.output_file_force = open(home + f"/Recordings/Force/Force_{self.base_file_name}.txt", 'a')  # Open file in append mode
        self.output_file_pulse = open(home + f"/Recordings/Pulse/Pulse_{self.base_file_name}.txt", 'a')  # Open file in append mode
        self.output_file_position = open(home + f"/Recordings/Position/Position_{self.base_file_name}.txt", 'a')  # Open file in append mode
        self.rate = rospy.Rate(10) #4 Hz
        self.counter = 0
        self.counter_ = 0

    def record_callback(self, data1, data2):
        current_Fz = data2.wrench.force.z
        current_pose_z = data1.transform.translation.z
        # rospy.loginfo("Received data: %s", current_Fz)
        if self.counter_ == 0:
            prev_Fz = current_Fz
            # print("prev_Fz:", prev_Fz)
            self.counter_ +=1
        
        if self.counter == 0:
            self.t1 = rospy.get_time()
            self.output_file_pulse.write(str(self.time_axis) + '\n')  # Write data to file
            self.output_file_pulse.flush()      

        t2 = rospy.get_time()
        diff_time = t2-self.t1
        if diff_time <= self.total_time:
            print("THz is recording: ", self.counter)
            self.output_file_force.write(str(current_Fz) + '\n')  # Write data to file
            self.output_file_force.flush()  # Flush buffer to ensure data is written immediately
            self.output_file_position.write(str(current_pose_z) + '\n')  # Write data to file
            self.output_file_position.flush()  # Flush buffer to ensure data is written immediately
            pulse_data = self.submit("GETLATESTPULSE", "d", 8*self.numpoints[0]) #array with the pulse data
            # print("size",sys.getsizeof(pulse_data))
            # print("counter",self.counter)
            self.output_file_pulse.write(str(pulse_data) + '\n')  # Write data to file
            self.output_file_pulse.flush()

        else:
            return_value = self.submit("STOP","?", 8)#Starting/Stopping Spectrometer if scanning
            # print("STOPPED")
            # self.disconnect()
        self.counter += 1

        


    def run(self):
        rospy.spin()
        self.rate.sleep()

    def __del__(self):
        return_value = self.submit("STOP","?", 8)
        # self.disconnect()
        self.output_file_force.close()  # Close file when the node is terminated
        self.output_file_pulse.close()
        self.output_file_position.close()
        
    def submit(self,command, type_id, n_bit):
        sock = socket.socket()
        sock.connect((self.ip_addr,self.port))
        sock.sendall(command.encode('utf-8'))
        time.sleep(0.075)
        return_value = [e[0] for e in list(struct.iter_unpack(type_id, sock.recv(n_bit)))]
        sock.close()
        return return_value
    
    
if __name__ == '__main__':

   #for GUI comment it out, otherwise uncomment for direct python running.
    # if len(sys.argv) != 2:
    #     print("Usage: python script.py <base_file_name>")
    #     sys.exit(1)

    base_file_name = sys.argv[1]
    print("*********Saving THz and Force data with base file name as "+ base_file_name + "for " + sys.argv[2] + " secs  ************")
    total_time = sys.argv[2]
    # ip = "10.216.47.23"
    ip = "192.170.10.10"
    port = 8001
    try:
        data_logger = THzROS(ip, port, base_file_name, total_time)
        data_logger.run()
    except rospy.ROSInterruptException:
        pass
