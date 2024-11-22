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
MID_ANGLE = 0.04
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
        self.ctrl_timer = self.create_timer(1/500, self.ctrl_callback)
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

        self.ctrl_update = False


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
    
    def ctrl_callback(self):
        self.ctrl_update = True

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
                                                  delta_t=1/400, 
                                                  show_animation=False)
        self.ros_manager = RosTopicManager()
        self.ros_manager_thread = threading.Thread(target=rclpy.spin, args=(self.ros_manager,), daemon=True)
        self.ros_manager_thread.start()
        self.running_flag = False
        self.turning_pid = PID(0.5, 0, 0.0001)
        
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
        X = np.zeros((4, 1))    # X = [x, x_dot, theta, theta_dot]
        U = np.zeros((1, 1))
        t0 = time.time()
        X_ref = np.zeros((4, 1))
        yaw_ref = 0.
        yaw_speed = 0.
        start_time = time.time()

        X_list = []
        U_list = []

        while self.running_flag:
            if self.ros_manager.ctrl_update:
                # print(self.ros_manager.imu_update)
                self.ros_manager.ctrl_update = False
                # print('imu false')
                t1 = time.time()
                dt = t1 - t0
                # print('freq:', 1/dt)
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

                motor_command_left = max(-1.5, min(motor_command_left, 1.5))
                motor_command_right = max(-1.5, min(motor_command_right, 1.5))
                # print(motor_command_left, motor_command_right)

                self.ros_manager.send_foc_command(motor_command_left, motor_command_right)

                if len(X_list)<=3001:
                    X_list.append(np.copy(X))
                    U_list.append(np.copy(U))
                    if len(X_list)==3000:
                        with open('crazydog_ws/src/lqr_control/lqr_control/log/log_x.plk', 'wb') as f1:
                            pickle.dump(X_list, f1)
                        with open('crazydog_ws/src/lqr_control/lqr_control/log/log_u.plk', 'wb') as f2:
                            pickle.dump(U_list, f2)
                        print('log save')
                        # X_list.clear()

            else:
                print('wait')
                pass
                

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