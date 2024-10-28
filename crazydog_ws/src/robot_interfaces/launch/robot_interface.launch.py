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
        Node(
            package='joy',
            executable='joy_node',
            name='joystick'
        ),
        Node(
            package='robot_interfaces',
            executable='joy_controller',
            name='joy_controller'
        ),
        Node(
            package='robot_interfaces',
            executable='unitree_pubsub',
            name='unitree_pubsub'
        ),
        # Node(
        #    package='robot_interfaces',
        #    executable='odom_pub',
        #    name='odom_pub'
        # ),
    ])