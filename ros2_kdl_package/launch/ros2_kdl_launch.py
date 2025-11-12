import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from pathlib import Path

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_kdl_package'),
        'config',
        'parameters.yaml'
    )
    
    default_log_folder = str(Path.home() / 'ros2_ws' / 'src' / 'hmwk2_plotting')
    
    log_folder_arg = DeclareLaunchArgument(
        'log_folder_path', 
        default_value=default_log_folder, 
        description='Cartella in cui salvare i file CSV'
    )
    
    cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface',
        default_value='velocity', 
        description='Command interface type (position, velocity, velocity_ctrl_null,vision, effort)'
    )
    
    # Camera bridge
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            '/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'
        ],
        output='screen'
    )

    kdl_node = Node(
        package='ros2_kdl_package',
        name='Iiwa_pub_sub',
        executable='ros2_kdl_node',
        parameters=[config, 
            {
                'cmd_interface': LaunchConfiguration('cmd_interface'),
                'log_folder_path': LaunchConfiguration('log_folder_path')  
            }],
        output='screen'
    )

    # Bridge to create a ROS 2 service for setting model pose in Gazebo
    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/world/default/set_pose@ros_gz_interfaces/srv/SetEntityPose'
        ],
        name='gz_set_pose_bridge'
    )

    
    
    return LaunchDescription([
        cmd_interface_arg,
        log_folder_arg,
        kdl_node,
        camera_bridge,
        set_pose_bridge
    ])