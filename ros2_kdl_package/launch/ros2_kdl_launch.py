import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('ros2_kdl_package'),
        'config',
        'parameters.yaml'
        )
        
    cmd_interface_arg = DeclareLaunchArgument(
        'cmd_interface',
        default_value='velocity',  # Il valore di default
        description='Command interface type (position, velocity, velocity_nullspace, effort)'
    )
        
    kdl_node=Node(
        package = 'ros2_kdl_package',
        name = 'Iiwa_pub_sub',
        executable = 'ros2_kdl_node',
        parameters = [config, 
        {
        	'cmd_interface': LaunchConfiguration('cmd_interface')
        }]
    )
    return LaunchDescription([
        cmd_interface_arg,
        kdl_node
    ])
