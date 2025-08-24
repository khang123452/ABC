from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    left_ip_arg = DeclareLaunchArgument('left_ip', default_value='192.168.88.40')
    right_ip_arg = DeclareLaunchArgument('right_ip', default_value='192.168.88.56')
    return LaunchDescription([
        left_ip_arg,
        right_ip_arg,
        Node(
            package='robopepp_bridge',
            executable='rtde_left_node',
            name='rtde_left',
            parameters=[{'ip': LaunchConfiguration('left_ip')}] ),
        Node(
            package='robopepp_bridge',
            executable='rtde_right_node',
            name='rtde_right',
            parameters=[{'ip': LaunchConfiguration('right_ip')}] ),
        Node(
            package='robopepp_bridge',
            executable='robopepp_node',
            name='robopepp',
            parameters=['config/robopepp_ur3.yaml'] )
    ])
