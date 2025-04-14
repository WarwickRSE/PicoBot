#!/usr/bin/env python3
import os
import rospy
import cv2
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import numpy as np
import json
import torch
from scipy.spatial.distance import cdist
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial import ConvexHull
import pyrealsense2 as rs2
import dynamic_reconfigure.client

class MarkerDetection:
    def __init__(self):
        rospy.init_node('green_marker_detection_with_cropbox', anonymous=True)
        # get current path of the script
        self.current_path = os.path.dirname(os.path.abspath(__file__))
        self.yolo_path =self.current_path + '/ultralytics_yolov5_master'
        self.weight_path = self.current_path + '/best.pt'
        
        # self.yolo_path = '/home/terabotics/stuff_ws/src/roi_detection/scripts/ultralytics_yolov5_master'
        # self.weight_path = '/home/terabotics/stuff_ws/src/roi_detection/scripts/best.pt'
        self.model = torch.hub.load(self.yolo_path,'custom',source='local',path=self.weight_path, device='cuda:0')
        
        self.bridge = CvBridge()

        self.pub_image = rospy.Publisher('detected_markers',Image,queue_size=1)
        # self.pub_pointcloud = rospy.Publisher('cropped_point_cloud', PointCloud2, queue_size=1)
        self.pub_marker = rospy.Publisher('hull_Center', PoseStamped, queue_size=1)

        self.point_cloud_sub = rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.pointcloud_callback)
        self.depth_sub  = rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/aligned_depth_to_color/camera_info', CameraInfo, self.camera_info_callback)
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.image_callback)
        
        

        self.current_frame = None
        self.current_pointcloud = None
        self.current_depth_image = None
        self.camera_intrinsics = None

        # Dynamic reconfigure server
        self.client = dynamic_reconfigure.client.Client("cropbox")
        
        # Initialize crop box parameters
        self.min_x = -0.01
        self.min_y = -0.01
        self.max_x = 0.01
        self.max_y = 0.01
        self.min_z = 0.0
        self.max_z = 0.5

        rospy.loginfo("Marker Detector Node Initialized")

    def camera_info_callback(self, data):
        self.camera_intrinsics = data.K
    def depth_callback(self, data):
        self.current_depth_image_ = data
        self.current_depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")

    def image_callback(self,data):
        self.current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        predictions = self.model(self.current_frame)
        results = json.loads(predictions.pandas().xyxy[0].to_json(orient="records"))
        centers = self.draw_bboxes(results, self.current_frame)
        if centers:
            # Filter centers based on distance from the main cluster
            filtered_centers = self.filter_centers(centers)
            points3D = self.draw_convex_hull(filtered_centers, self.current_frame)

            # self.CropBox(points3D)

            self.pub_image.publish(self.bridge.cv2_to_imgmsg(self.current_frame, encoding="bgr8"))
        else:
            self.default_CropBox()


    def pointcloud_callback(self, msg):
        # Store the latest point cloud data
        self.current_pointcloud = msg

    def draw_bboxes(self, results, current_frame):
        centers = []
        for bounding_box in results:
            if bounding_box['confidence'] > 0.5:
                start_point = (int(bounding_box["xmin"]), int(bounding_box["ymin"]))
                end_point = (int(bounding_box["xmax"]), int(bounding_box["ymax"]))
                cv2.rectangle(current_frame, start_point, end_point, (0, 255, 0), thickness=2)

                #  Calculate the center of the bounding box
                center_x = int((bounding_box["xmin"] + bounding_box["xmax"]) / 2)
                center_y = int((bounding_box["ymin"] + bounding_box["ymax"]) / 2)
                centers.append((center_x, center_y))
                cv2.circle(current_frame, (center_x, center_y), 3, (0, 0, 255), -1)
        return centers
    
    def filter_centers(self, centers, distance_threshold=200):
        """Filter centers based on their distance from the main cluster, distance_threshold is in pixels."""
        if len(centers) < 2:
            return centers

        # Compute the distance matrix
        distances = cdist(centers, centers)

        # Compute the mean distance to the nearest neighbor
        mean_distances = np.mean(np.sort(distances)[:, 1:], axis=1)
        # print(mean_distances)
        # Filter out points that are too far from their neighbors
        main_cluster = [centers[i] for i in range(len(centers)) if mean_distances[i] < distance_threshold]
        
        return main_cluster
    
    def draw_convex_hull(self, centers, current_frame):
        points = np.array(centers)
        if len(points)>3:
            hull = cv2.convexHull(points)

            M = cv2.moments(hull)
            hcx = int(M['m10'] / M['m00'])
            hcy = int(M['m01'] / M['m00'])
            ref_point = np.array([hcx, hcy])
            angles = np.arctan2(hull[:,0,1] - ref_point[1], hull[:,0,0] - ref_point[0])

            sorted_indices = np.argsort(angles)
            sorted_hull = hull[sorted_indices]
            sorted_points = points[sorted_indices]

            cv2.polylines(current_frame, [sorted_hull], True, (255, 0, 0), 2)

            

            # convert sorted_points to 3D coordinates
            fx, fy = self.camera_intrinsics[0], self.camera_intrinsics[4]
            cx, cy = self.camera_intrinsics[2], self.camera_intrinsics[5]
        
            z_hc = self.current_depth_image[hcy, hcx]/1000
            x_hc = (hcx - cx) * z_hc / fx
            y_hc = (hcy - cy) * z_hc / fy

            self.CropBox(x_hc, y_hc, z_hc)

            # publish this x,y,z as Pose Marker
            pose = PoseStamped()
            pose.header.frame_id = "camera_color_optical_frame"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = x_hc
            pose.pose.position.y = y_hc
            pose.pose.position.z = z_hc
            pose.pose.orientation.w = 1.0
            self.pub_marker.publish(pose)
            
            # print(sorted_points)
            points3D =[]
            # for i in range(len(sorted_points)):
            #     x, y = sorted_points[i, 0], sorted_points[i, 1]
            #     # print(x,y)
            #     z = self.current_depth_image[y, x]/1000
            #     x3d = (x - cx) * z / fx
            #     y3d = (y - cy) * z / fy
            #     points3D.append([x3d, y3d, z])
            # # print(points3D)
            return points3D
        else:
            pass
            
        
    # def CropBox(self, points3D):
    #     points3D = np.array(points3D)
    #     # print(points3D)
    #     xfilter_limit_min = min(points3D[:,0])
    #     xfilter_limit_max = max(points3D[:,0])
    #     yfilter_limit_min = min(points3D[:,1])
    #     yfilter_limit_max = max(points3D[:,1])
    #     zfilter_limit_min = min(points3D[:,2])
    #     zfilter_limit_max = max(points3D[:,2])
    #     # print(xfilter_limit_min, xfilter_limit_max, yfilter_limit_min, yfilter_limit_max, zfilter_limit_min, zfilter_limit_max)
    #     params = {'min_x':xfilter_limit_min, 'max_x':xfilter_limit_max,'min_y': yfilter_limit_min,'max_y': yfilter_limit_max, 'max_z': zfilter_limit_max, 'min_z':zfilter_limit_min}
    #     config = self.client.update_configuration(params)

    def CropBox(self, x,y,z):
        # print(points3D)
        xfilter_limit_min = x - 0.0125
        xfilter_limit_max = x + 0.0125
        yfilter_limit_min = y - 0.0125
        yfilter_limit_max = y + 0.0125
        zfilter_limit_min = z - 0.005
        zfilter_limit_max = z + 0.005
        # print(xfilter_limit_min, xfilter_limit_max, yfilter_limit_min, yfilter_limit_max, zfilter_limit_min, zfilter_limit_max)
        params = {'min_x':xfilter_limit_min, 'max_x':xfilter_limit_max,'min_y': yfilter_limit_min,'max_y': yfilter_limit_max,'min_z': zfilter_limit_min, 'max_z': zfilter_limit_max}
        config = self.client.update_configuration(params)

    def default_CropBox(self):
        params = {'min_x':  self.min_x,
                    'max_x':self.max_x,
                    'min_y':self.min_y,
                    'max_y':self.max_y,
                    'min_z':self.min_z,
                    'max_z':self.max_z}
        config = self.client.update_configuration(params)
        



if __name__ == '__main__':
    detector = MarkerDetection()
    rospy.spin()
      