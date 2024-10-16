from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_interfaces'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
         glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anson',
    maintainer_email='anson@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'foc_command_sub = robot_interfaces.foc_command_sub:main',
            'foc_data_pub = robot_interfaces.foc_data_pub:main',
            'imu_data_pub = robot_interfaces.imu_data_pub:main',
            'joy_controller = robot_interfaces.joy_controller:main',
            'unitree_pubsub = robot_interfaces.unitree_pubsub:main',
            'odom_pub = robot_interfaces.odom_pub:main',
        ],
    },
)
