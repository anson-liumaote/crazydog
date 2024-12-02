from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('robot_interfaces'),
        'config',
        'robotv1.yaml'
    )
    print(config)
    return LaunchDescription([
        Node(
            package='robot_interfaces',
            executable='foc_command_sub',
            name='foc_command_sub'
        ),
        Node(
            package='robot_interfaces',
            executable='foc_data_pub',
            name='foc_data_pub',
            parameters= [config]
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
        # Node(
        #     package='robot_interfaces',
        #     executable='unitree_pubsub',
        #     name='unitree_pubsub',
        #     parameters=[config]
        # ),
        Node(
            package='robot_interfaces',
            executable='unitree_left_pub',
            name='unitree_left_pub',
            parameters=[config]
        ),
        Node(
            package='robot_interfaces',
            executable='unitree_right_pub',
            name='unitree_right_pub',
            parameters=[config]
        ),
        Node(
            package='robot_interfaces',
            executable='jointstate_pub',
            name='jointstate_pub',
            parameters=[config]
        ),
        Node(
           package='robot_interfaces',
           executable='odom_pub',
           name='odom_pub',
           parameters=[config]
        ),
    ])