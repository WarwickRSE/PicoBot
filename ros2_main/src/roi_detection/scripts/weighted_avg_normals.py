import rospy
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, PoseStamped, Pose
from geometry_msgs.msg import PoseStamped
import message_filters
import numpy as np
import math

class WeightedAvgNormals:
    def __init__(self):
        rospy.init_node('weighted_avg_normals', anonymous=True)
        self.d_max=0.4
        self.a = 0.3
        self.b= 4
        self.pub = rospy.Publisher('final_pose_output', PoseArray, queue_size=1)
        self.sub_poses = message_filters.Subscriber('/pose_output', PoseArray, queue_size=1)
        self.sub_center = message_filters.Subscriber('/hull_Center', PoseStamped, queue_size=1)
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_poses, self.sub_center], 1, 0.1, allow_headerless=True)
        ts.registerCallback(self.callback)


    def callback(self, poses, center):
        pose_array = PoseArray()
        pose_array.header = poses.header
        pose_array.poses = poses.poses
        weights = []
        sum_weight = []
        x,y,z=0,0,0
        q=[]
        # get weighted average of positions and orientations
        if poses is None:
            rospy.logwarn("No poses received")
            pass
        else:
            for i in range(len(pose_array.poses)):
                #  d = sqrt((x - x)^2 + (y - y)^2 + (z - z)^2)
                d = np.sqrt(math.pow((center.pose.position.x - pose_array.poses[i].position.x), 2)
                            + math.pow((center.pose.position.y - pose_array.poses[i].position.y), 2)
                            + math.pow((center.pose.position.z - pose_array.poses[i].position.z), 2))
                weight = np.round(self.a*np.exp(1/(self.b*(math.pow(d,2)-math.pow(self.d_max,2)))),5)
                weights.append(weight)
                # print("intial",x)
                x = x + weight*pose_array.poses[i].position.x
                y = y + weight*pose_array.poses[i].position.y
                z = z + weight*pose_array.poses[i].position.z
                q.append([pose_array.poses[i].orientation.w, pose_array.poses[i].orientation.x, pose_array.poses[i].orientation.y, pose_array.poses[i].orientation.z])
                # sum_weight.append(weight)

            total_weight = np.sum(weights)
            x = np.float64(x/total_weight)
            y = np.float64(y/total_weight)
            z = np.float64(z/total_weight)
            # print("values are:",x,y,z)
            q = np.array(q)
            weights = np.array(weights)
        
            q_avgd = self.quatWAvgMarkley(q, weights)
            if q_avgd is not None:
                avg_pose = PoseArray()
                avg_pose.header = pose_array.header
                avg_pose.header.stamp = rospy.Time.now()
                P = Pose()
                # avg_pose.pose.position.x = x
                # avg_pose.pose.position.y = y
                # avg_pose.pose.position.z = z
                P.position.x = center.pose.position.x
                P.position.y = center.pose.position.y
                P.position.z = center.pose.position.z
                P.orientation.w = q_avgd[0]
                P.orientation.x = q_avgd[1]
                P.orientation.y = q_avgd[2]
                P.orientation.z = q_avgd[3]
                avg_pose.poses.append(P)
                self.pub.publish(avg_pose)



    def quatWAvgMarkley(self,Q, weights):
        '''
        Averaging Quaternions.

        Arguments:
            Q(ndarray): an Mx4 ndarray of quaternions.
            weights(list): an M elements list, a weight for each quaternion.
        '''
        # print(Q)
        # Form the symmetric accumulator matrix
        A = np.zeros((4, 4))
        M = Q.shape[0]
        wSum = 0
        if M == 0:
            rospy.logwarn("No quaternions received")
            pass
        else:
            for i in range(M):
                q = Q[i, :]
                w_i = weights[i]
                A += w_i * (np.outer(q, q)) # rank 1 update
                wSum += w_i

            # scale
            A /= wSum

            # Get the eigenvector corresponding to largest eigen value
            return np.linalg.eigh(A)[1][:, -1]


if __name__ == '__main__':
    try:
        WeightedAvgNormals()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass