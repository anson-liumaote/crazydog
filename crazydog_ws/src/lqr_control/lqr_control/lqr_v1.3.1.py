import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
from geometry_msgs.msg import Twist
import threading
import time
import sys
# sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/pidControl/pidControl/modules/unitree_actuator_sdk/lib')
# from crazydog_ws.src.pidControl.pidControl.modules.unitree_actuator_sdk import *
import traceback
import math
import numpy as np
from src.lqr_control.lqr_control.utils.LQR_pin import InvertedPendulumLQR
from sensor_msgs.msg import Imu

import unitree_motor_command as um

import pickle

WHEEL_RADIUS = 0.08     # m
WHEEL_MASS = 0.695  # kg
WHEEL_DISTANCE = 0.355
URDF_PATH = "/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control/robot_models/big bipedal robot v1/urdf/big bipedal robot v1.urdf"
MID_ANGLE = 0.059
TORQUE_CONSTRAIN = 1.5

class PID:
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral = 0
        self.previous_error = 0

    def update(self, target, now, dt):

        error = target - now
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        self.previous_error = error
        return output
    
class focMotor():
    def __init__(self):
        self.angle = 0.0    # rad
        self.speed = 0.0    # rpm
        self.current = 0.0  
        self.temperature = 0.0  # degree C

class RosTopicManager(Node):
    def __init__(self):
        super().__init__('ros_topic_manager')
        self.foc_data_subscriber = self.create_subscription(Float32MultiArray,'foc_msg', self.foc_callback, 1)
        self.imu_subscriber = self.create_subscription(Imu,'handsfree/imu', self.imu_callback, 1)
        self.vel_subscriber = self.create_subscription(Twist,'cmd_vel', self.vel_callback, 1)
        self.foc_command_publisher = self.create_publisher(Float32MultiArray, 'foc_command', 1)
        self.imu_monitor = self.create_publisher(Float32, 'imu_monitor', 1)
        self.tau_monitor = self.create_publisher(Float32, 'tau_monitor', 1)
        self.foc_right = focMotor()
        self.foc_left = focMotor()
        # self.pitch_last = 0
        # self.pitch_dot = 0
        self.row = 0
        self.row_last = 0
        self.row_dot = 0
        self.dt = 1/300
        self.joy_linear_vel = 0.
        self.joy_angular_vel = 0.



    def foc_callback(self, msg):
        if msg.data[0] == 513.:   # motor left
            self.foc_left.angle = msg.data[1]
            self.foc_left.speed = msg.data[2]
            self.foc_left.current = msg.data[3]
            self.foc_left.temperature = msg.data[4]
        elif msg.data[0] == 514.: # motor right
            self.foc_right.angle = -msg.data[1]
            self.foc_right.speed = -msg.data[2]
            self.foc_right.current = -msg.data[3]
            self.foc_right.temperature = msg.data[4]
        else:
            self.get_logger().error('foc callback id error')
    
    def send_foc_command(self, current_left, current_right):
        msg = Float32MultiArray()
        torque_const = 0.3  # N-m/A
        msg.data = [current_left/torque_const, -current_right/torque_const]
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
        # self.pitch = -(math.asin(2 * (qua_w * qua_y - qua_z * qua_x)) - self.pitch_bias) 
        self.row_dot = msg.angular_velocity.x
        # self.row_dot = (self.row - self.row_last) / self.dt
        self.row_last = self.row
        # self.pitch_dot = (self.pitch - self.pitch_last) / self.dt
        # self.pitch_last = self.pitch
        # self.pitch_dot = msg.angular_velocity.y

    def get_orientation(self):
        return -self.row, -self.row_dot
    
    def vel_callback(self, msg):
        self.joy_linear_vel = msg.linear.x
        self.joy_angular_vel = msg.angular.z

    def get_joy_vel(self):
        return self.joy_linear_vel, -self.joy_angular_vel

class robotController():
    def __init__(self) -> None:
        rclpy.init()
        # K: [[ 2.97946709e-07  7.36131891e-05 -1.28508761e+01 -4.14185118e-01]]
        Q = np.diag([1e-9, 0.1, 1.0, 1e-4])       # 1e-9, 1e-9, 0.01, 1e-6; 1e-9, 0.1, 1.0, 1e-4
        R = np.diag(np.diag([2e-2]))
        q = np.array([0., 0., 0., 0., 0., 0., 1.,
                            0., -1.18, 2.0, 1., 0.,
                            0., -1.18, 2.0, 1., 0.])
        self.lqr_controller = InvertedPendulumLQR(pos=q, 
                                                  urdf=URDF_PATH, 
                                                  wheel_r=WHEEL_RADIUS, 
                                                  M=WHEEL_MASS, Q=Q, R=R, 
                                                  delta_t=1/500, 
                                                  show_animation=False)
        self.ros_manager = RosTopicManager()
        self.ros_manager_thread = threading.Thread(target=rclpy.spin, args=(self.ros_manager,), daemon=True)
        self.ros_manager_thread.start()
        self.running_flag = False
        self.turning_pid = PID(0.5, 0, 0.0001)

    def init_unitree_motor(self):
        self.unitree = um.unitree_communication('/dev/unitree-l')
        self.MOTOR1 = self.unitree.createMotor(motor_number = 1,initalposition = 0.669,MAX=8.475,MIN=-5.364)
        self.MOTOR2 = self.unitree.createMotor(motor_number = 2,initalposition = 3.815,MAX=26.801,MIN=-1)
        self.unitree2 = um.unitree_communication('/dev/unitree-r')
        self.MOTOR4 = self.unitree2.createMotor(motor_number = 4,initalposition = 1.247,MAX=5.364,MIN=-8.475)
        self.MOTOR5 = self.unitree2.createMotor(motor_number = 5,initalposition = 5.046,MAX=1,MIN=-26.801)    
        self.unitree.inital_all_motor()
        self.unitree2.inital_all_motor()

    def locklegs(self):
        while self.MOTOR1.data.q >= self.MOTOR1.inital_position_cheak_point + 0.33*6.33 and self.MOTOR4.data.q  <= self.MOTOR4.inital_position_cheak_point -0.33*6.33  :
            self.unitree.position_force_velocity_cmd(motor_number = 1,kp = 0,kd = 0.1, position = 0 ,torque = 0, velocity = 0.01)
            self.unitree2.position_force_velocity_cmd(motor_number = 4 ,kp = 0,kd = 0.1, position = 0 ,torque = 0, velocity=-0.01)
        time.sleep(0.01)
        for i in range(36):                        
            self.unitree.position_force_velocity_cmd(motor_number = 1,kp = i,kd = 0.12, position = self.MOTOR1.inital_position_cheak_point + 0.33*6.33)
            self.unitree2.position_force_velocity_cmd(motor_number = 4 ,kp = i,kd = 0.12, position = self.MOTOR4.inital_position_cheak_point - 0.33*6.33)
            time.sleep(0.1)
        while self.MOTOR2.data.q >= self.MOTOR2.inital_position_cheak_point + 0.33*6.33*1.6 and self.MOTOR5.data.q  <= self.MOTOR5.inital_position_cheak_point - 0.33*6.33*1.6:
            self.unitree.position_force_velocity_cmd(motor_number = 2,kp = 0,kd = 0.16, position = 0 ,torque = 0, velocity = 0.01)
            self.unitree2.position_force_velocity_cmd(motor_number = 5 ,kp = 0,kd = 0.16, position = 0 ,torque = 0, velocity=-0.01)
        time.sleep(0.01)
        for i in range(36):                        
            self.unitree.position_force_velocity_cmd(motor_number = 2,kp = i,kd = 0.15, position = self.MOTOR2.inital_position_cheak_point + 0.6*6.33*1.6)
            self.unitree2.position_force_velocity_cmd(motor_number = 5 ,kp = i,kd = 0.15, position = self.MOTOR5.inital_position_cheak_point - 0.6*6.33*1.6)
            time.sleep(0.1)

    def startController(self):
        self.prev_pitch = 0
        self.lqr_thread = threading.Thread(target=self.controller)
        self.running_flag = True
        self.lqr_thread.start()

    def get_yaw_speed(self, speed_left, speed_right):
        delta_speed = (speed_left-speed_right)* (2*np.pi*WHEEL_RADIUS)/60
        yaw_speed = delta_speed / WHEEL_DISTANCE
        return yaw_speed

    def controller(self):
        self.ros_manager.get_logger().info('controller start')
        X_list = []
        U_list = []
        X = np.zeros((4, 1))    # X = [x, x_dot, theta, theta_dot]
        U = np.zeros((1, 1))
        t0 = time.time()
        X_ref = np.zeros((4, 1))
        yaw_ref = 0.
        yaw_speed = 0.
        start_time = time.time()
        

        while self.running_flag:
            t1 = time.time()
            dt = t1 - t0
            t0 = t1
            X_ref[2, 0], yaw_ref = self.ros_manager.get_joy_vel()
            X_ref[2, 0] += MID_ANGLE
            X_last = np.copy(X)
            foc_status_left, foc_status_right = self.ros_manager.get_foc_status()
            # get motor data
            X[1, 0] = (foc_status_left.speed + foc_status_right.speed) / 2 * (2*np.pi*WHEEL_RADIUS)/60
            X[0, 0] = X_last[0, 0] + X[1, 0] * dt * WHEEL_RADIUS     # wheel radius 0.08m
            # get IMU data
            X[2, 0], X[3, 0] = self.ros_manager.get_orientation()
            # X[2, 0], _ = self.ros_manager.get_orientation()
            # X[3, 0] = (X[2, 0] - X_last[2, 0]) / dt
            if abs(X[2, 0]) > math.radians(15):     # constrain
                # U[0, 0] = 0.0
                self.ros_manager.send_foc_command(0.0, 0.0)
                continue
            
            # get u from lqr
            U = np.copy(self.lqr_controller.lqr_control(X, X_ref))
            # print(X)
            # print('u:', U[0, 0])
            # print('freq:', 1/dt)
            # time.sleep(3e-3)
            # print(X)
            # print(X_desire[2,0])
            yaw_speed = self.get_yaw_speed(foc_status_left.speed, foc_status_right.speed)
            yaw_torque = self.turning_pid.update(yaw_ref, yaw_speed, dt)

            if (time.time()-start_time) < 3:
                yaw_torque = 0.0
            motor_command_left = U[0, 0] + yaw_torque
            motor_command_right = U[0, 0] - yaw_torque
            # print(yaw_torque)

            motor_command_left = max(-TORQUE_CONSTRAIN, min(motor_command_left, TORQUE_CONSTRAIN))
            motor_command_right = max(-TORQUE_CONSTRAIN, min(motor_command_right, TORQUE_CONSTRAIN))

            self.ros_manager.send_foc_command(motor_command_left, motor_command_right)
            print(motor_command_left, motor_command_right)
            if len(X_list)<=3001:
                X_list.append(X)
                U_list.append(U)
                if len(X_list)==3000:
                    with open('crazydog_ws/src/lqr_control/lqr_control/log/log.plk', 'wb') as f:
                        pickle.dump(X_list, f)
                    print('log save')
                    X_list.clear()
            
            
        # self.ros_manager.send_foc_command(0.0, 0.0)

    def disableController(self):
        self.running_flag = False
        self.ros_manager.send_foc_command(0.0, 0.0)
        if self.lqr_thread is not None:
            self.lqr_thread.join()
        self.ros_manager.get_logger().info("disable controller")
    
    def disableUnitreeMotor(self):
        self.unitree.disableallmotor()
        self.unitree2.disableallmotor()



def main(args=None):
    robot = robotController()
    command_dict = {
        "d": robot.disableUnitreeMotor,
        "i": robot.init_unitree_motor,
        'l': robot.locklegs,
        "start": robot.startController,
    }

    while True:
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                robot.disableController()
                break
        except KeyboardInterrupt:
            robot.disableController()
            break
        # except Exception as e:
        #     traceback.print_exc()
        #     break


if __name__ == '__main__':
    main()