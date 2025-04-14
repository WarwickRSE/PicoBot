# cv_trails


## Marker-based segmentation and pose estimation


## Offline Merker detection using YOLOV5s model 
### Install YOLOv5 requirements 
Look at link : https://github.com/ultralytics/yolov5
>`$ git clone https://github.com/ultralytics/yolov5`
`$ cd yolov5`
`$ pip install -r requirements.txt`

2. Ensure `best.pt` file is in `cv_trails/scripts ` This file contains the weights of the trained network
3. Ensure `ultralytics_yolov5_master` folder has been cloned correctly and lies inside `cv_trails/scripts `
### Execute the code:
   `rosrun cv_trails green_marker_detection.py`


## Online Marker detection: 
### Install :
1. RobotFlow :
   ` pip install roboflow `
3. Packaging:
   ` pip install packaging `
5. numpy v1.21:
   `pip install numpy==1.21`

### Execute: 
- filters on camera
 `roslaunch pcl_tester filters_on_camera.launch`
 - Marker detection/segmentation
 `rosrun cv_trails roboflow_detection`
 - Compute Poses
 `rosrun testing ComputePoses`
 - Final weighted averaged pose
 `rosrun cv_trails normal_estimation`


## Marker-less Pose Estimation:

### Install:
1. MediaPipe :
   Check https://developers.google.com/mediapipe/framework/getting_started/install  and https://developers.google.com/mediapipe/solutions/vision/face_landmarker/python for full documentation and usage

   `python -m pip install mediapipe`
   
2. Splines :
   `pip install splines`

3. Open3D :
   `pip install open3d `
   Check : http://www.open3d.org/docs/release/getting_started.html

    
### Select points to scan :

1. On the terminal Go to :
` cd stuff_ws/src/cv_trails/scripts `

2. Select the points to scan :
` python face_mesh_initialisation_multiple_points.py `

Refer to https://github.com/google/mediapipe for full details on the face mesh generated and the indices at different points on the face

4. On the image window, LEFT CLICK on the points on the face mesh to select points
   
6. RIGHT CLICK once done
   
7. GO TO :

` cd home/shruti/gace_mesh_initialisation folder ` and check `points_to_scan.txt` file to ensure the point indices selected have been saved

(Alternatively, you can directly put the indices here and save them for tests and debugging) 

### Launch Camera :


1. `roslaunch realsense2_camera rs_camera.launch enable_pointcloud:=true filters:=spatial,temporal align_depth:=True clip_distance:=0.8 `

To learn about the parameters such as `clip_distance`, `filters: temporal,spatial` and `align_depth` look at https://dev.intelrealsense.com/docs/python2

2. Launch `rqt` and in `spatial` change `hole_fill` to `unlimited` and `temporal` to `1 in 8 frames`


### Launch Cropbox Filter : 
`roslaunch tera_iiwa_ros filters_on_camera `



### Crop PointCloud around Face : 
` rosrun cv_trails clipCloudToFace `

This will crop the pointcloud around the detected face for each camera frame. This publishes an output to /cropbox/output


## To scan multiple points :



### Start Track points to scan
1. Go To : 
`cd stuff_ws/src/cv_trails/scripts`

2. RUN:
`python face_mesh_track_multiple_points.py`

### To compute all surface normals (This does not run if Cropbox filter is not correctly set) : 

` rosrun tera_iiwa ros ComputePoses `

### To compute the weighted averaged normal at the selected point:

1. GO TO :

`cd stuff_ws/src/cv_trails/scripts`

2. RUN: 
`python normal_estimation_multiple_points.py `

### To Visualise results use: 
` rviz ` for visualising the results from the camera and the codes and ` rqt_graph ` for visualizing nodes that are running

## To scan on the sides of the nose: 
CAUTION: Currently the probe size is too big to scan on the side of the nose. Run the following steps in order to get pose any of the points on the side of the nose:
1. GO TO :

`cd stuff_ws/src/cv_trails/scripts`

2. RUN: 
`python nose_tracking.py `

This code will publish a pose array to `/final_pose_output`. You can then directly follow the steps below to transform this to the robot frame and command to the robot
 

## To send Pose to the Robot:

### Launch the robot

Read instructions on https://github.com/anubhav-dogra/tera_iiwa_ros

They are also summarised here: 

1. Connect the ethernet cable and check IP address

2. TURN ON Robot, Choose FRIOVERLAYGRIPPER from applications

3. On the terminal run:

` roslaunch iiwa_driver iiwa_bringup.launch controller:=CartesianImpedance_trajectory_controller model:=iiwa14 `

### To get Hand eye calibration:

` rosrun tera_iiwa_ros eye_in_hand `


### To transform pose from camera frame to robot frame:


` rosrun tera_iiwa_ros tf_listener_test ` 


### To start trajectory planner:


` roslaunch cartesian_trajectory_generator trajectory_generator.launch `

### To plan and send commands: 

` rosrun tera_iiwa_ros plan_send_cartesian_commands `


### Send target pose to the robot:

 ` rosrun tera_iiwa_ros send_target_pose `

### Send robot to home position:
>rostopic pub --once /cartesian_trajectory_generator/new_goal geometry_msgs/PoseStamped "header:
>  seq: 0
>  stamp:
>    secs: 0
>    nsecs: 0
>  frame_id: 'world'
>pose:
  position:
    x: -0.62
    y: 0.0
    z: 0.26
  orientation:
    x: 0.7
    y: 0.7
    z: 0.0
    w: 0.0" 


### Get iiwa_tool_link_ee poses :

` roslaunch tera_iiwa_ros get_wrench.launch `


### To launch force controller : 
`roslaunch tera_iiwa_ros force_controller.launch`


### Example commands to Record Rosbags (To record results) :

` rosbag record -O robot_motion_pt_151.bag /tf `

` rosbag record -O pose_normal_pt.bag --duration=60 /tf_array_out `