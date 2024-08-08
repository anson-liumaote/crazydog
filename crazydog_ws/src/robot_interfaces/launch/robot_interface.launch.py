from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_interfaces',
            executable='foc_command_sub',
            name='foc_command_sub'
        ),
        Node(
            package='robot_interfaces',
            executable='foc_data_pub',
            name='foc_data_pub'
        ),
        Node(
            package='robot_interfaces',
            executable='imu_data_pub',
            name='imu_data_pub'
        ),
    ])