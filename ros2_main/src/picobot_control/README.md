# picobot_control

## To set up iiwa
 - This package launches the robot controller in torque control modes. That means you are sending torques commands now rather than position or velocities: GOTTA be more CAREFULL
 - If you'r:
 	- using End-effector, make sure you are using `FRIOverlayGripper` on the SmartPad.
 	- if no end-effector, use `FRIOverlay`.
 	Otherwise robot can go crazy due to inaccurate EXTERNAL TORQUES on the joint sensors. 
 - When using torque mode or `Cartesian Impedance controller`: (V.Important). Stiffness values should be 0. 
 


### Gazebo
`roslaunch iiwa_gazebo iiwa_gazebo.launch controller:=CartesianImpedance_trajectory_controller model:=iiwa14`

This launch `iiwa_setup` and `iiwa_tools` to setup the robot in the space and configure servers like FK, IK, Jacobians, Gravity, etc. to use in the simulations or others. Check `iiwa_ros` for more details

`iiwa_setup` contains a launch file to upload robot urdf to the parameter server. 
if you are changing the end-effector/ no end effector: launch edits example: iiwa14_tool_upload.launch/iiwa14_upload.launch

### on KUKA LBR
`roslaunch iiwa_driver iiwa_bringup.launch controller:=CartesianImpedance_trajectory_controller model:=iiwa14`

This launch `iiwa_bringup`, `iiwa_setup` and `iiwa_tools` to setup the robot in the space and configure servers like FK, IK, Jacobians, Gravity, etc. to use in the simulations or others. Mostly these things are handeled through KUKA controller itself.


## Command robot

### Two ways:
 - Best is to use `cartesian_trajectory_controller`:

 `roslaunch picobot_control cam_viz.launch`

 `rostopic pub --once cartesian_trajectory_generator/new_goal...` tab tab

- #### to move the robot
 `rosrun picobot_control plan_send_cartesian_commands`


-  AVOID USING THIS> CHECK POSE FILTER IN CONFIG IN iiwa_ros/iiwa_control/config/iiwa_control.yaml
  `rostopic pub --once /iiwa/CartesianImpedance_trajectory_controller/reference_pose... ` tab tab
 MAKE SURE, COMMANDING POSE IS NOT VERY FAR. 
 
### Safety
 - Safety configuration is equipped with
 	* Joint limits
 	* Cartesian Speed limit (200mm/sec)
	* Max. external torques on the joints (10Nm)

 - Limit to the commanding torque values Through URDF torque limits (Need to check again)
 - Can also impose max. velocities to the joints through Safety configuration and URDF limits. (NOT YET IMplEMENTED)

### Get Wrench
`roslaunch picobot_control get_wrench.launch`

 in simulation:
`roslaunch picobot_control get_wrench_sim.launch`
 



### Icon
```
[Desktop Entry]
Version=1.0
Type=Application
Name=Terabotics
Icon=/home/robothz/Pictures/logo.svg
Exec=bash -c "source /opt/ros/noetic/setup.bash ; source /home/robothz/stuff_ws/devel/setup.bash; export ROS_IP=192.170.10.1 ; export ROS_MASTER_URL=HTTP://$ROS_IP:11311 ; python3 /home/robothz/gui_ws/Terabotics_switches.py ; read -p press"
Terminal=true
Type=Application

```
Save this file as Filename.desktop
then mark its properties as executable. 