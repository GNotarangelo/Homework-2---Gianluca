1. (a)  To start the simulation we always run:  
 "ros2 launch iiwa_bringup iiwa.launch.py use_sim:="true" command_interface:="velocity" robot_controller:="velocity_controller""  
   and we make sure to start the gazebo simulation with the play button

1. (b) To use the velocity control with null space optimization:
 "ros2 launch ros2_kdl_package ros2_kdl_launch.py cmd_interface:=velocity_ctrl_null"  

  The node automatically logs positions and commanded velocities to csv files so that we can later use a python script in order to visualize them:  

  -move to the directory hmwk2_plotting and then run:
  "python3 plot_vision.py" (this same script will be used to visualize the other controllers' velocity commands)  


1. (c) To use the action controller:   
"ros2 launch ros2_kdl_package ros2_kdl_launch.py cmd_interface:=action"   

and then on an other terminal use the following command line action  request:  


ros2 action send_goal /execute_trajectory ros2_kdl_package/action/TrajectoryExecution '{
  traj_duration: 10.0,
  acc_duration: 2.0,
  end_position: [0.0, 0.7, 0.4],
  traj_type: "linear",
  s_type: "cubic"
}'  


We also created a trajectory client node which can be run from cli and send the trajectory. To run this,in another command line:  

"ros2 run ros2_kdl_package trajectory_client"  




  2. (a-b) To use the vision controller:  

   "ros2 launch ros2_kdl_package ros2_kdl_launch.py cmd_interface:=vision". 

  It is also needed to start the aruco node (in an other terminal):   

  "ros2 run aruco_ros single --ros-args --remap image:=/camera --remap camera_info:=/camera_info -p marker_id:=6 -p marker_size:=0.10 -p camera_frame:='camera_link' -p marker_frame:='marker_frame' -p use_sim_time:=true"    



  2. (c) To use the service call in order to move the tag:  

  ros2 service call /world/default/set_pose ros_gz_interfaces/srv/SetEntityPose "{
  entity: {
    name: 'arucotag',
    type: 2
  },
  pose: {
    position: {x: 0.8, y: 0.2, z: 1.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}"    


