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

class RosTopicManager(Node):
    def __init__(self):
        super().__init__('ros_topic_manager')
        self.foc_data_subscriber = self.create_subscription(Float32MultiArray,'foc_msg', self.foc_callback, 1)
        self.imu_subscriber = self.create_subscription(Imu,'handsfree/imu', self.imu_callback, 5)
        self.vel_subscriber = self.create_subscription(Twist,'cmd_vel', self.vel_callback, 1)
        self.vel_subscriber = self.create_subscription(String,'body_pose', self.body_pose_callback, 1)
        self.foc_command_publisher = self.create_publisher(Float32MultiArray, 'foc_command', 1)
        self.imu_monitor = self.create_publisher(Float32, 'imu_monitor', 1)
        self.tau_monitor = self.create_publisher(Float32, 'tau_monitor', 1)
        # self.ctrl_timer = self.create_timer(1/500, self.ctrl_callback)
        self.foc_right = focMotor()
        self.foc_left = focMotor()
        # self.pitch_last = 0
        # self.pitch_dot = 0
        self.row = 0
        self.row_last = 0
        self.row_dot = 0
        self.yaw = 0
        self.yaw_dot = 0
        self.dt = 1/300
        self.joy_linear_vel = 0.
        self.joy_angular_vel = 0.

        self.wheel_coordinate = [0.033-0.0752, -0.2285]  # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))

        self.ctrl_condition = threading.Condition()

        self.unitree_command_sub = self.create_subscription(
                LowState,
                'unitree_status',
                self.status_callback,
                1)
        self.motor_states = LowState()
        self.motor_cmd_pub = self.create_publisher(LowCommand, 'unitree_command', 1)

        self.ta = time.time()

    def body_pose_callback(self, msg):
        if msg.data == "up":
            self.wheel_coordinate[1] -= 0.0001
        elif msg.data == "down":
            self.wheel_coordinate[1] += 0.0001
        elif msg.data == "left":
            self.wheel_coordinate[0] += 0.0001
        elif msg.data == "right":
            self.wheel_coordinate[0] -= 0.0001

    def status_callback(self, msg_list):
        self.motor_states = msg_list.motor_state
        with self.ctrl_condition:
            self.ctrl_condition.notify()
        

    def foc_callback(self, msg):
        if msg.data[0] == 513.:   # motor left
            self.foc_left.angle = -msg.data[1]
            self.foc_left.speed = -msg.data[2]
            self.foc_left.current = -msg.data[3]
            self.foc_left.temperature = msg.data[4]
        elif msg.data[0] == 514.: # motor right
            self.foc_right.angle = msg.data[1]
            self.foc_right.speed = msg.data[2]
            self.foc_right.current = msg.data[3]
            self.foc_right.temperature = msg.data[4]
        else:
            self.get_logger().error('foc callback id error')
    
    def send_foc_command(self, current_left, current_right):
        msg = Float32MultiArray()
        torque_const = 0.247  # 0.3 N-m/A
        msg.data = [-current_left/torque_const, current_right/torque_const]
        self.foc_command_publisher.publish(msg)

    def get_foc_status(self):
        return self.foc_left, self.foc_right
    
    def imu_callback(self, msg):
        qua_x = msg.orientation.x
        qua_y = msg.orientation.y
        qua_z = msg.orientation.z
        qua_w = msg.orientation.w
        # *(180/math.pi)+1.5
        t0 = +2.0 * (qua_w * qua_x + qua_y * qua_z)
        t1 = +1.0 - 2.0 * (qua_x * qua_x + qua_y * qua_y)
        self.row = math.atan2(t0, t1)
        self.yaw = math.atan2(2 * (qua_w * qua_z + qua_x * qua_y), 1 - 2 * (qua_y**2 + qua_z**2))
        

        # self.pitch = -(math.asin(2 * (qua_w * qua_y - qua_z * qua_x)) - self.pitch_bias) 
        self.row_dot = msg.angular_velocity.x
        self.yaw_dot = msg.angular_velocity.z
        # self.row_dot = (self.row - self.row_last) / self.dt
        self.row_last = self.row
        # print(-self.row)
        # self.pitch_dot = (self.pitch - self.pitch_last) / self.dt
        # self.pitch_last = self.pitch
        # self.pitch_dot = msg.angular_velocity.y

        # with self.ctrl_condition:
        #     self.ctrl_condition.notify()
        

    def get_orientation(self):
        return -self.row, -self.row_dot,
    
    def vel_callback(self, msg):
        self.joy_linear_vel = msg.linear.x
        self.joy_angular_vel = msg.angular.z

    def get_joy_vel(self):
        return self.joy_linear_vel, -self.joy_angular_vel
    
    # def ctrl_callback(self):
    #     self.ctrl_update = True


class focMotor():
    def __init__(self):
        self.angle = 0.0    # rad
        self.speed = 0.0    # rpm
        self.current = 0.0  
        self.temperature = 0.0  # degree C
