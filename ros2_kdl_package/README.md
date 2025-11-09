# ros2_kdl_package

## :package: About

This package contains the tutorial code to create and run your ROS2 C++ node using KDL. It is supposed to be used together with the [ros2_iiwa package](https://github.com/RoboticsLab2025/ros2_iiwa) 

## :hammer: Build
Clone this package in the `src` folder of your ROS 2 workspace. Check for missing dependencies
```
rosdep install -i --from-path src --rosdistro humble -y
```
Build your new package
```
colcon build --packages-select ros2_kdl_package
```
Source the setup files
```
. install/setup.bash
```

## :white_check_mark: Usage
Run the node
```
ros2 run ros2_kdl_package ros2_kdl_node
```

By default the node publishes joint position commands. To use the velocity commands 
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity
```
in this case the robot must be launched with the velocity interface
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="velocity" robot_controller:="velocity_controller"
```

To use the effort commands 
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=effort
```
in this case the robot must be launched with the velocity interface
```
ros2 launch iiwa_bringup iiwa.launch.py command_interface:="effort" robot_controller:="effort_controller"
```

Other parameters are
```
traj_type:=linear|circular, s_type:=trapezoidal|cubic
```
For instance, you can execute a cubic polinomial circular trajectory by
```
ros2 run ros2_kdl_package ros2_kdl_node --ros-args -p cmd_interface:=velocity -p traj_type:=circular -p s_type:=cubic
```

