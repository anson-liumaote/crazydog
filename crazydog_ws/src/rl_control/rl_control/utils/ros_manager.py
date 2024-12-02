import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import threading
import math
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
import time
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from scipy.spatial.transform import Rotation as R

INIT_JOINT_POS = []

class RosManager(Node):
    def __init__(self):
        super().__init__('ros_topic_manager')
        # self.foc_data_subscriber = self.create_subscription(Float32MultiArray,'foc_msg', self.foc_callback, 1)
        # self.imu_subscriber = self.create_subscription(Imu,'handsfree/imu', self.imu_callback, 5)
        self.vel_subscriber = self.create_subscription(Twist,'cmd_vel', self.vel_callback, 1)
        self.vel_subscriber = self.create_subscription(String,'body_pose', self.body_pose_callback, 1)
        self.jointstate_subscriber = self.create_subscription(JointState,'jointstate', self.jointstate_callback, 1)
        self.odom_subscriber = self.create_subscription(Odometry,'odom', self.odom_callback, 1)
        self.foc_command_publisher = self.create_publisher(Float32MultiArray, 'foc_command', 1)
        self.imu_monitor = self.create_publisher(Float32, 'imu_monitor', 1)
        self.tau_monitor = self.create_publisher(Float32, 'tau_monitor', 1)
        self.control_timer = self.create_timer(0.02, self.control_timer_callback) 
        self.row = 0
        self.row_last = 0
        self.row_dot = 0
        self.joy_linear_vel = 0.
        self.joy_angular_vel = 0.

        self.wheel_coordinate = [0.033-0.0752, -0.2285]  # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))

        self.ctrl_condition = threading.Condition()

        self.unitree_command_sub = self.create_subscription(
                LowState,
                'unitree_status_all',
                self.status_callback,
                1)
        self.motor_states = LowState()
        self.motor_cmd_pub = self.create_publisher(LowCommand, 'unitree_command', 1)

        self.joint_state = JointState()
        self.odom = Odometry()

        # Initialize observation variables
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        self.projected_gravity = np.zeros(3)  # Update this based on your sensor data or estimation
        self.velocity_commands = np.zeros(3)  # Define as needed
        self.joint_pos = np.zeros(4)
        self.joint_vel = np.zeros(6)

    def control_timer_callback(self):
        # notify inference ctrl
        with self.ctrl_condition:
            self.ctrl_condition.notify()

    def jointstate_callback(self, msg: JointState):
        # Assuming the first two are positional joints, and next four are velocity joints
        self.joint_pos = np.array([msg.position[1], msg.position[4], msg.position[2], msg.position[5]])-np.array([1.271, 1.271, -2.12773, -2.12773])
        self.joint_vel = np.array([msg.velocity[1], msg.velocity[2], msg.velocity[6], msg.velocity[4], msg.velocity[5], msg.velocity[7]])
        # old api
        self.joint_state = msg
        

    def odom_callback(self, msg: Odometry):
        # Extract linear and angular velocity from odometry message
        self.base_lin_vel[0] = msg.twist.twist.linear.x
        self.base_lin_vel[1] = msg.twist.twist.linear.y
        self.base_lin_vel[2] = msg.twist.twist.linear.z
        self.base_ang_vel[0] = msg.twist.twist.angular.x
        self.base_ang_vel[1] = msg.twist.twist.angular.y
        self.base_ang_vel[2] = msg.twist.twist.angular.z
        # Update other inputs as needed, e.g., projected gravity
        self.projected_gravity = self.get_projected_gravity(msg)
        # old api
        self.odom = msg
    
    def body_pose_callback(self, msg):
        if msg.data == "up":
            self.wheel_coordinate[1] -= 0.0001
        elif msg.data == "down":
            self.wheel_coordinate[1] += 0.0001
        elif msg.data == "left":
            self.wheel_coordinate[0] += 0.0001
        elif msg.data == "right":
            self.wheel_coordinate[0] -= 0.0001

    def vel_callback(self, msg: Twist):
        # self.joy_linear_vel = msg.linear.x
        self.joy_angular_vel = msg.angular.z
        # Smooth velocity
        alpha = 0.1
        self.joy_linear_vel = (alpha * msg.linear.x) + ((1 - alpha) * self.joy_linear_vel)
        # self.joy_angular_vel = (alpha * msg.angular.z) + ((1 - alpha) * self.joy_angular_vel)

    def status_callback(self, msg_list: LowState):
        self.motor_states = msg_list.motor_state
    
    def send_foc_command(self, tau_left, tau_right):
        msg = Float32MultiArray()
        # torque_const = 0.247  # 0.3 N-m/A
        # msg.data = [-tau_left/torque_const, tau_right/torque_const]
        msg.data = [-float(tau_left), float(tau_right)]
        self.foc_command_publisher.publish(msg)
    
    def get_joint_pos(self, joint_name):
        try:
            index = self.joint_state.name.index(joint_name)
            return self.joint_state.position[index]
        except:  
            self.get_logger().error(f'get_joint_pos {joint_name} error.')
            return None
        
    def get_joint_vel(self, joint_name):
        try:
            index = self.joint_state.name.index(joint_name)
            return self.joint_state.velocity[index]
        except:  
            self.get_logger().error(f'get_joint_vel {joint_name} error.')
            return None

    def get_pitch_orientation(self):
        roll, pitch, yaw = self.euler_from_quaternion(self.odom.pose.pose.orientation.x, 
                                                      self.odom.pose.pose.orientation.y, 
                                                      self.odom.pose.pose.orientation.z, 
                                                      self.odom.pose.pose.orientation.w)
        return pitch, self.odom.twist.twist.angular.y

    def get_yaw_orientation(self):
        roll, pitch, yaw = self.euler_from_quaternion(self.odom.pose.pose.orientation.x, 
                                                      self.odom.pose.pose.orientation.y, 
                                                      self.odom.pose.pose.orientation.z, 
                                                      self.odom.pose.pose.orientation.w)
        return yaw, self.odom.twist.twist.angular.z
    
    def get_joy_vel(self):
        return self.joy_linear_vel, self.joy_angular_vel
    
    def get_linear_vel_x(self):
        return self.odom.twist.twist.linear.x
    
    def get_linear_pos_xy(self):
        return [self.odom.pose.pose.position.x, self.odom.pose.pose.position.y]

    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians
    
    def get_projected_gravity(self, odom_msg:Odometry):
        # Extract the quaternion (x, y, z, w) from odometry orientation
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        
        # Define gravity vector in the world frame
        gravity_world = np.array([0, 0, -1])  # Standard gravity

        # Convert quaternion to rotation matrix
        rotation = R.from_quat([qx, qy, qz, qw])
        rotation_matrix = rotation.as_matrix()

        # Rotate the gravity vector into the robot's frame
        projected_gravity = rotation_matrix.T @ gravity_world  # Transpose to rotate into base frame

        return projected_gravity