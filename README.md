Command to launch simulation in vision control mode:
ros2 launch iiwa_bringup iiwa.launch.py use_sim:=true vision_control:=true

Command to launch aruco single node with correct remapping to connect to the camera output:
ros2 run aruco_ros single   --ros-args   --remap image:=/camera   --remap camera_info:=/camera_info   -p marker_id:=6   -p marker_size:=0.10   -p camera_frame:='camera_link' -p marker_frame:='marker_frame'

