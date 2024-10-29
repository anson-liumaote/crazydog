import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray, Float32
import threading

import time
import sys
# sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/pidControl/pidControl/modules/unitree_actuator_sdk/lib')
# from crazydog_ws.src.pidControl.pidControl.modules.unitree_actuator_sdk import *
import traceback
import math
import numpy as np
from MPC_pin import InvertedPendulumMPC
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt

import unitree_motor_command as um
from mpc_test import *

WHEEL_RADIUS = 0.08     # m
WHEEL_MASS = 0.695  # kg
URDF_PATH = "/home/crazydog/crazydog/crazydog_ws/src/mpc_control/mpc_control/robot_models/big bipedal robot v1/urdf/big bipedal robot v1.urdf"

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
        # self.row_dot = msg.angular_velocity.x
        self.row_dot = (self.row - self.row_last) / self.dt
        self.row_last = self.row
        # self.pitch_dot = (self.pitch - self.pitch_last) / self.dt
        # self.pitch_last = self.pitch
        # self.pitch_dot = msg.angular_velocity.y

    def get_orientation(self):
        return -self.row, -self.row_dot

class robotController():
    def __init__(self) -> None:
        rclpy.init()
        
        self.ros_manager = RosTopicManager()
        self.ros_manager_thread = threading.Thread(target=rclpy.spin, args=(self.ros_manager,), daemon=True)
        self.ros_manager_thread.start()
        self.running_flag = False

        

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
        while self.MOTOR1.data.q >= self.MOTOR1.inital_position + 0.33*6.33 and self.MOTOR4.data.q  <= self.MOTOR4.inital_position  :
            self.unitree.position_force_velocity_cmd(motor_number = 1,kp = 0,kd = 0.1, position = 0 ,torque = 0, velocity = 0.01)
            self.unitree2.position_force_velocity_cmd(motor_number = 4 ,kp = 0,kd = 0.1, position = 0 ,torque = 0, velocity=-0.01)
        time.sleep(0.01)
        for i in range(36):                        
            self.unitree.position_force_velocity_cmd(motor_number = 1,kp = i,kd = 0.12, position = self.MOTOR1.inital_position + 0.33*6.33)
            self.unitree2.position_force_velocity_cmd(motor_number = 4 ,kp = i,kd = 0.12, position = self.MOTOR4.inital_position - 0.33*6.33)
            time.sleep(0.1)
        while self.MOTOR2.data.q >= self.MOTOR2.inital_position + 0.33*6.33*1.6 and self.MOTOR5.data.q  <= self.MOTOR5.inital_position - 0.33*6.33*1.6:
            self.unitree.position_force_velocity_cmd(motor_number = 2,kp = 0,kd = 0.16, position = 0 ,torque = 0, velocity = 0.01)
            self.unitree2.position_force_velocity_cmd(motor_number = 5 ,kp = 0,kd = 0.16, position = 0 ,torque = 0, velocity=-0.01)
        time.sleep(0.01)
        for i in range(36):                        
            self.unitree.position_force_velocity_cmd(motor_number = 2,kp = i,kd = 0.15, position = self.MOTOR2.inital_position + 0.6*6.33*1.6)
            self.unitree2.position_force_velocity_cmd(motor_number = 5 ,kp = i,kd = 0.15, position = self.MOTOR5.inital_position - 0.6*6.33*1.6)
            time.sleep(0.1)

    def startController(self):
        self.prev_pitch = 0
        self.lqr_thread = threading.Thread(target=self.controller)
        self.running_flag = True
        self.lqr_thread.start()

    def controller(self):
        self.ros_manager.get_logger().info('controller start')
        t0 = time.time()
        
        while self.running_flag:
            prob = run_mpc()
            x_init.value = x0
            # print('TIME: ', round(i*dt, 2), 'STATES: ', [round(state, 2) for state in x0])
            prob.solve(solver=cp.OSQP, warm_start=True)
            
            motor_command = u[:, 0].value
            # self.ros_manager.send_foc_command(motor_command, motor_command)
            print('motor', motor_command)
            t1 = time.time()
            dt = t1 - t0
            t0 = t1
            print('freq:', 1/dt)
            print('x0:', x0)
            x0_last = np.copy(x0) 
            foc_status_left, foc_status_right = self.ros_manager.get_foc_status()
            # get motor data
            x0[0,] = (foc_status_left.speed + foc_status_right.speed) / 2 * (2*np.pi*WHEEL_RADIUS)/60
            x0[0, ] = x0_last[0, ] + x0_last[1,] * dt * WHEEL_RADIUS     # wheel radius 0.08m
            # get IMU data
            x0[2,], x0[3,] = self.ros_manager.get_orientation()
            # X[2, 0], _ = self.ros_manager.get_orientation()
            # X[3, 0] = (X[2, 0] - X_last[2, 0]) / dt
            if abs(x0[2,]) > math.radians(30):     # constrain
                # U[0, 0] = 0.0
                self.ros_manager.send_foc_command(0.0, 0.0)
                continue
            
            # self.mpc_controller.update(X)
            # get u from mpc
            # U = np.copy(self.mpc_controller.solve())
            # print(x0)
            # print('u:', U[0, 0])
            
            # time.sleep(3e-3)
            # print(X)
            # motor_command = U[0, 0]
            # self.ros_manager.send_foc_command(motor_command, motor_command)

            # imu_msg = Float32()
            # imu_msg.data = X[1, 0]
            # self.ros_manager.imu_monitor.publish(imu_msg)

            # tau_msg = Float32()
            # tau_msg.data = U[0, 0]
            # self.ros_manager.tau_monitor.publish(tau_msg)
            
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
        # "get": robot_motor.getControllerPIDParam,
        # "clear": robot_motor.mc.cleanerror,
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