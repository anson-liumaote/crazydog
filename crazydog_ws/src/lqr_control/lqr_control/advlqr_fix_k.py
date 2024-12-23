import rclpy
import threading
import time
import math
import numpy as np
from utils.LQR_pin import InvertedPendulumLQR
from utils.pid import PID
from utils.ros_manager import RosTopicManager
# from utils.motor_getready import disableUnitreeMotor, init_unitree_motor, locklegs, enable
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
import pickle
from datetime import datetime
import os
from utils import urdf_loader
import matplotlib.pyplot as plt

WHEEL_RADIUS = 0.07     # m
WHEEL_MASS = 0.695  # kg
WHEEL_DISTANCE = 0.355
URDF_PATH = "/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control/robot_models/big bipedal robot v1/urdf/big bipedal robot v1.urdf"
MID_ANGLE = 0.0   #0.05
TORQUE_CONSTRAIN = 1.5
# MOTOR_INIT_POS = [None, 0.478, 1.190, None, 1.247+2*math.pi, 0.944]    # for unitree motors
INIT_ANGLE = [-2.42, 2.6]
# LOCK_POS = [None, 2.76, 9.88, None, 5.44, -3.10]    # -2.75, 2.0
THIGH_LENGTH = 0.215
CALF_LENGTH = 0.215

MOTOR_ORIGIN_POS = [0.0, -4.5, 27.8, 0.0, 14.8, -25.6, 0.0, 0.0]
SCALE = [6.33, 6.33, 6.33*1.6, 6.33, -6.33, -6.33*1.6, 1.0, 1.0]

class robotController():
    def __init__(self) -> None:
        rclpy.init()
        # K: [[ 2.97946709e-07  7.36131891e-05 -1.28508761e+01 -4.14185118e-01]]
        Q = np.diag([0., 10., 100., 0.1])       # 1e-9, 0.1, 1.0, 1e-4
        R = np.diag(np.diag([2.5]))   #0.02
        q = np.array([0., 0., 0., 0., 0., 0., 1.,
                            0., -1.18, 2.0, 1., 0.,
                            0., -1.18, 2.0, 1., 0.])
        adva_k = np.array([[-7.1176   ,-0.5863   ,-0.4869   ,-0.8220    ,2.9448    ,0.4212],
                          [7.2087    ,0.3191    ,0.4390    ,0.6566    ,6.6901    ,0.6554]])
        #[-7.2468 ,-0.9553 ,-0.9533 ,-1.4048 ,3.5574 ,0.5730],
        #[4.1840 ,0.1985 ,-0.0127  ,-0.0743 ,7.8870 ,0.7830]
        # ([[-6.8033 ,-0.7575 ,-0.7423 ,-1.1602 ,3.3082 ,0.5328],
        #  [4.4794  ,0.0089 ,0.1080  ,0.0777 ,7.7223 ,0.7510]])
        self.robot = urdf_loader.loadRobotModel(urdf_path=URDF_PATH)
        self.robot.pos = q
        self.com, self.l_bar = self.robot.calculateCom(plot=False)
        mass = self.robot.calculateMass()
        self.lqr_thread = None
        self.lqr_controller = InvertedPendulumLQR(pos=q, 
                                                  urdf=URDF_PATH, 
                                                  wheel_r=WHEEL_RADIUS, 
                                                  M=WHEEL_MASS, Q=Q, R=R, 
                                                  delta_t=1/250, 
                                                  show_animation=False,
                                                  m=mass,
                                                  l_bar=self.l_bar,
                                                  dynamic_K = True,
                                                  max_l=0.46,
                                                  min_l=0.07,
                                                  slice_w=0.03,
                                                  advance_lqr_K=adva_k)
        self.lqr_controller.change_K(self.l_bar)
        # l_bar from 0.1 to 0.37
        self.hip_P = 20
        self.hip_D = 0.55
        self.turn_P = 0.05
        self.turn_D = 0.0002
        self.knee_P = 0.1
        self.knee_D = 0.05

        self.phi_list = []
        self.theta_list = []
        self.X_list = []
        self.time_list = []
        self.x_dot_list = []
        self.x_dot_ref_list = []
        self.acc_x = []
        self.hip_theta = []

    def enable_ros_manager(self):
        self.ros_manager = RosTopicManager()
        self.ros_manager_thread = threading.Thread(target=rclpy.spin, args=(self.ros_manager,), daemon=True)
        self.ros_manager_thread.start()
        self.running_flag = False
        self.adva_running_flag = False
        self.turning_pid2 = PID(self.turn_P, 0, self.turn_D)
        self.hip_pid = PID(self.hip_P ,0, self.hip_D)
        self.knee_pid1 = PID(self.knee_P,0, self.knee_D)
        self.knee_pid2 = PID(self.knee_P,0, self.knee_D)
        self.cmd_list = LowCommand()
        time.sleep(2)

    def check_target_pos(self, j_name):
        if self.ros_manager.get_joint_pos(j_name) >= 0.8324:
            return True
        else: 
            return False   
            
    def set_motor_cmd(self, motor_number, kp, kd, position, torque=0, velocity=0, scaling=False):
        if scaling==False:
            torque = max(-5, min(torque, 5))
            cmd = MotorCommand()
            cmd.q = float(position)
            cmd.dq = float(velocity)
            # cmd.q, cmd.dq = self.inverse_scaling(position, velocity)
            cmd.tau = float(torque)
            cmd.kp = float(kp)
            cmd.kd = float(kd)
            self.cmd_list.motor_cmd[motor_number] = cmd
        else:
            # torque = max(-10, min(torque, 15))
            cmd = MotorCommand()
            cmd.q = float(position) * SCALE[motor_number] + MOTOR_ORIGIN_POS[motor_number]
            cmd.dq = float(velocity) * SCALE[motor_number]
            # cmd.q, cmd.dq = self.inverse_scaling(position, velocity)
            cmd.tau = max(-1.0, min(float(torque)/SCALE[motor_number], 1.0))
            cmd.kp = float(kp)
            cmd.kd = float(kd)
            self.cmd_list.motor_cmd[motor_number] = cmd

    def init_unitree_motor(self):
        if self.check_target_pos("thigh_l") and self.check_target_pos("thigh_r"):
            while self.check_target_pos("thigh_l") and self.check_target_pos("thigh_r"):
                # self.set_motor_cmd(motor_number=1, kp=2, kd=0.02, position=self.ros_manager.motor_states[1].q-0.1, torque=0, velocity=0)
                # self.set_motor_cmd(motor_number=4, kp=2, kd=0.02, position=self.ros_manager.motor_states[4].q+0.1, torque=0, velocity=0)
                self.set_motor_cmd(motor_number=1, kp=0, kd=0.2, position=0, torque=0, velocity=-0.5)
                self.set_motor_cmd(motor_number=4, kp=0, kd=0.2, position=0, torque=0, velocity=0.5)
                self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
                time.sleep(0.001)
            self.set_motor_cmd(motor_number=1, kp=6., kd=0.1, position=0.8324, torque=0, velocity=0, scaling=True)
            # self.set_motor_cmd(motor_number=2, kp=6., kd=0.1, position=self.ros_manager.motor_states[2].q, torque=0, velocity=0)
            self.set_motor_cmd(motor_number=4, kp=6., kd=0.1, position=0.8324, torque=0, velocity=0, scaling=True)
            # self.set_motor_cmd(motor_number=5, kp=6., kd=0.1, position=self.ros_manager.motor_states[5].q, torque=0, velocity=0)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
        else:
            print("inital fail")
    
    def plotResult(self):
        
        plt.figure(figsize=(20,12))

        plt.subplot(6, 1, 1)
        plt.plot(self.time_list, self.phi_list, label="body tilt", color='blue')
        plt.title('lqr Controller')
        plt.ylabel('Phi(rad)')
        plt.xlabel('Time')
        plt.grid(True)

        plt.subplot(6, 1, 2)
        plt.plot(self.time_list, self.X_list, label="robot displacement", color='orange')
        plt.ylabel('Position(m)')
        plt.xlabel('Time')
        plt.grid(True)

        plt.subplot(6, 1, 3)
        plt.plot(self.time_list, self.x_dot_list, label="robot velocity", color='green')
        plt.plot(self.time_list, self.x_dot_ref_list, color='blue')
        plt.ylabel('velocity(m/s)')
        plt.xlabel('Time')
        plt.grid(True)
 
        plt.subplot(6, 1, 4)
        plt.plot(self.time_list, self.theta_list, label="hip angle", color='red')
        plt.ylabel('Theta(rad)')
        plt.xlabel('Time')
        plt.grid(True)

        plt.subplot(6, 1, 5)
        plt.plot(self.time_list, self.acc_x, label="imu_acc_x", color='yellow')
        plt.ylabel('(m/s^2)')
        plt.xlabel('Time')
        plt.grid(True)

        plt.subplot(6, 1, 6)
        plt.plot(self.time_list, self.hip_theta, label="hip_theta", color='yellow')
        plt.ylabel('rad')
        plt.xlabel('Time')
        plt.grid(True)
        # plt.plot()
        plt.tight_layout()
        plt.savefig('advlqr_controller_output.png')
        print("The image has been saved as 'pid_controller_output.png'")
    
    def inverse_kinematics(self, x, y, L1=THIGH_LENGTH, L2=CALF_LENGTH):
        # 計算 d    
        d = np.sqrt(x**2 + y**2)
        # 計算 theta2
        cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        theta2 = np.arccos(cos_theta2)
        
        # 計算 theta1
        theta1 = np.arctan2(y, x) - np.arctan2(L2 * np.sin(theta2), L1 + L2 * np.cos(theta2))
    
        return theta1, theta2
    
    def get_angle_error(self, axis):
        theta1, theta2 = self.inverse_kinematics(axis[0], axis[1])
        theta2 = max(0, min(theta2, 2.618))
        self.robot.pos = np.array([0., 0., 0., 0., 0., 0., 1.,
                                0., theta1+1.57, theta2, 1., 0.,
                                0., theta1+1.57, theta2, 1., 0.])
        self.com, self.l_bar = self.robot.calculateCom()
        theta1_err = theta1 - INIT_ANGLE[0]
        theta2_err = theta2 - INIT_ANGLE[1]
        return theta1_err, theta2_err

    def locklegs(self):
        # self.ros_manager.wheel_coordinate = [-0.0639-ORIGIN_BIAS[0], -0.003637-ORIGIN_BIAS[1]]
        # theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)     # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
        # self.lqr_controller.change_K(self.l_bar)
        thigh_angle = 1.216 # 1.271
        calf_angle = -2.14  # -2.12773
        while self.ros_manager.get_joint_pos('thigh_l') <= thigh_angle and self.ros_manager.get_joint_pos('thigh_r') <= thigh_angle:
            self.set_motor_cmd(motor_number=1, kp=0, kd=0.05, position=0, torque=0, velocity=0.2, scaling=False)
            self.set_motor_cmd(motor_number=4, kp=0, kd=0.05, position=0, torque=0, velocity=-0.2, scaling=False)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.001)
        for i in range(20):                        
            self.set_motor_cmd(motor_number=1, kp=i, kd=0.12, position=thigh_angle, scaling=True)
            self.set_motor_cmd(motor_number=4, kp=i, kd=0.12, position=thigh_angle, scaling=True)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)
        while self.ros_manager.get_joint_pos('calf_l') <= calf_angle and self.ros_manager.get_joint_pos('calf_r') <= calf_angle:
            self.set_motor_cmd(motor_number=2, kp=0, kd=0, position=0, torque=0.7, velocity=0, scaling=False)
            self.set_motor_cmd(motor_number=5 ,kp=0, kd=0, position=0, torque=-0.7, velocity=0, scaling=False)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.001)
        for i in range(20):                        
            self.set_motor_cmd(motor_number=2, kp=i, kd=0.15, position=calf_angle, scaling=True)
            self.set_motor_cmd(motor_number=5, kp=i, kd=0.15, position=calf_angle, scaling=True)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)
        
    
    # def standup(self):
    #     self.ros_manager.wheel_coordinate = [0.0348784-0.0752, -0.10413465]
    #     theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)     # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
    #     self.lqr_controller.change_K(self.l_bar)
    #     self.startController()
    #     time.sleep(0.3)
    #     self.set_motor_cmd(motor_number=1, kp=5, kd=0.12, position=MOTOR_INIT_POS[1]-theta1_err*6.33)
    #     self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
    #     time.sleep(0.1)
    #     self.set_motor_cmd(motor_number=4, kp=5, kd=0.12, position=MOTOR_INIT_POS[4]+theta1_err*6.33)
    #     self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
    #     time.sleep(0.1)
    #     self.set_motor_cmd(motor_number=2, kp=10, kd=0.12, position=MOTOR_INIT_POS[2]-theta2_err*6.33*1.6)
    #     self.set_motor_cmd(motor_number=5, kp=10, kd=0.12, position=MOTOR_INIT_POS[5]+theta2_err*6.33*1.6)
    #     self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
        

    # def update_pose(self):
    #     theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)
    #     self.set_motor_cmd(motor_number=1, kp=10, kd=0.12, position=MOTOR_INIT_POS[1]-theta1_err*6.33)
    #     self.set_motor_cmd(motor_number=4, kp=10, kd=0.12, position=MOTOR_INIT_POS[4]+theta1_err*6.33)
    #     self.set_motor_cmd(motor_number=2, kp=10, kd=0.12, position=MOTOR_INIT_POS[2]-theta2_err*6.33*1.6)
    #     self.set_motor_cmd(motor_number=5, kp=10, kd=0.12, position=MOTOR_INIT_POS[5]+theta2_err*6.33*1.6)
    #     # print(theta1_err, theta2_err)
    #     self.ros_manager.motor_cmd_pub.publish(self.cmd_list)

    def calibrate_com(self):
        oMi = self.robot.getOmi()
        err = oMi - self.com[0]
        # print(err)
        # print(l_bar)
        self.ros_manager.wheel_coordinate[0] -= err*0.5

    def startadvaController(self):
        self.advalqr_thread = threading.Thread(target=self.adva_controller)
        if self.running_flag == True:
            self.running_flag = False
        self.adva_running_flag = True
        self.advalqr_thread.start()

    def startController(self):
        self.prev_pitch = 0
        self.lqr_thread = threading.Thread(target=self.controller)
        self.running_flag = True
        self.lqr_thread.start()

    def get_yaw_speed(self, speed_left, speed_right):

        delta_speed = (speed_left-speed_right)* (2*np.pi*WHEEL_RADIUS)/60
        yaw_speed = delta_speed / WHEEL_DISTANCE
        return yaw_speed
    
    def F_torque(self, knee_angle, F):

        torque = F*math.cos(math.pi/2+knee_angle/2)*CALF_LENGTH

        return torque
    
    def kalman_filter_data_fusion(self, dt, x, P): 
        sigma_v = 0.01 #速度噪聲方差
        sigma_a = 0.1 #加速度噪聲方差
        sigma_process = 1 #過程噪聲方差
        F = np.array([[1, dt],[0, 1]]) #狀態轉移矩陣
        H = np.eye(2) #量測矩陣
        Q = np.array([[sigma_v, 0],[0, sigma_a]]) #量測協方差矩陣
        R = np.array([[sigma_process, 0],[0, sigma_process]]) #過程協方差矩陣
        # gamma = np.array([[0.5*dt**2],[dt]])
        #預測步驟
        x_hat = F @ x #先驗估計
        P = F @ P @ F.T + Q #先驗誤差協方差
        #校正步驟
        K = P @ H.T @ np.linalg.inv(H @ P @ H.T + R) #get kalman gain
        x = x_hat + K @ (x - H @ x_hat) #後驗估計
        P = (np.eye(2) - K @ H) @ P #更新誤差協方差
        return x, P

    def adva_controller(self):
        X = np.zeros((6, 1))  # X = [theta, d_theta, x, d_x, phi, d_phi]
        U = np.zeros((2, 1))
        X_ref = np.zeros((6, 1))
        x = np.array([[0],[0]]) #kalman起始x
        P_1 = np.array([[0.1, 0],[0, 0.1]]) #kalman起始P
        t0 = time.time()
        start_time = time.time()
        leg_bias = 0.150  
        disp_s = 0    
        L_knee_angle_inital = 1.12
        R_knee_angle_inital = 0.874
        # Leg_desire = 0.215
        # feed_forward = 55.625

        while self.adva_running_flag==True and self.running_flag==False:
            with self.ros_manager.ctrl_condition:
                self.ros_manager.ctrl_condition.wait()
            current_time = time.time() - start_time
            t1 = time.time()
            dt = t1 - t0
            t0 = t1
            speed, yaw_ref = self.ros_manager.get_joy_vel()
            speed = speed*15
            if abs(X_ref[2, 0]-X[2, 0]) > 1.5: #限制防止位移過度累加
                speed = 0
            disp_s = disp_s + speed*dt
            X_ref[2, 0] = disp_s
            yaw_ref = yaw_ref*5
            # print(X_ref[3, 0],yaw_ref)
            X_last = np.copy(X)
            foc_status_left, foc_status_right = self.ros_manager.get_foc_status()
            ws_r = foc_status_right.speed * (2*np.pi*WHEEL_RADIUS)/60
            ws_l = foc_status_left.speed * (2*np.pi*WHEEL_RADIUS)/60
            x[0, 0] = (ws_l + ws_r)/2
            x[1, 0] = self.ros_manager.get_linear_acc()
            x, P_1 = self.kalman_filter_data_fusion(dt, x, P_1)
            X[3, 0] = x[0, 0]
            X[4, 0], X[5, 0], roll = self.ros_manager.get_orientation()
            X[4, 0], X[5, 0] = -X[4, 0], -X[5, 0]
            X[2, 0] = X_last[2, 0] + X[3, 0] * dt
            Lleg_position = (12.786 - self.ros_manager.motor_states[4].q)/6.33 - math.radians(60)-X[4, 0]-leg_bias
            Rleg_position = -(-4.486 - self.ros_manager.motor_states[1].q)/6.33 - math.radians(60)-X[4, 0]-leg_bias
            X[0, 0] = (Lleg_position+Rleg_position)/2
            X[1, 0] = (-self.ros_manager.motor_states[4].dq+self.ros_manager.motor_states[1].dq)/(6.33*2)

            L_knee_angle = (self.ros_manager.motor_states[2].q - L_knee_angle_inital)/(6.33*1.6)
            R_knee_angle = (R_knee_angle_inital - self.ros_manager.motor_states[5].q)/(6.33*1.6)
            avg_knee_angle = (L_knee_angle + R_knee_angle)/2
            L_leg_long = math.sin(L_knee_angle/2)*(THIGH_LENGTH*2)
            R_leg_long = math.sin(R_knee_angle/2)*(THIGH_LENGTH*2)

            U = np.copy(self.lqr_controller.advance_lqr_control(X,X_ref))

            yaw_speed = self.get_yaw_speed(foc_status_left.speed, foc_status_right.speed)
            target = yaw_ref
            output = self.turning_pid2.update(target,yaw_speed, dt)
            angle_error = Lleg_position - Rleg_position
            output2 = self.hip_pid.update(0, angle_error, dt)
            U_wr = U[0, 0] - output
            U_wl = U[0, 0] + output
            U_tr = U[1, 0] + output2
            U_tl = U[1, 0] - output2
            
            # print(U[0, 0], U[1, 0])
            motor_command_right = U_wr
            motor_command_left = U_wl
            thigh_command_right = U_tr
            thigh_command_left = U_tl
            # soft constrain
            motor_command_left = max(-3, min(motor_command_left, 3))
            motor_command_right = max(-3, min(motor_command_right, 3))
            thigh_command_right = max(-2.5, min(thigh_command_right, 2.5))
            thigh_command_left = max(-2.5, min(thigh_command_left, 2.5))

            self.set_motor_cmd(motor_number=4,torque=-thigh_command_right/6.33)
            self.set_motor_cmd(motor_number=1,torque=thigh_command_left/6.33)
            # self.set_motor_cmd(motor_number=2,torque=L_knee_torque/(6.33*1.6))
            # self.set_motor_cmd(motor_number=5,torque=-R_knee_torque/(6.33*1.6))

            # self.ros_manager.send_foc_command(motor_command_left , motor_command_right)
            # self.ros_manager.motor_cmd_pub.publish(self.cmd_list)

            self.phi_list.append(X[4, 0])
            self.theta_list.append(X[0, 0])
            self.x_dot_list.append(X[3, 0])
            self.X_list.append(X[2, 0])
            self.x_dot_ref_list.append(speed)
            self.acc_x.append(x[1, 0])
            self.hip_theta.append(self.ros_manager.motor_states[4].q)
            self.time_list.append(current_time)

            if abs(X[4, 0]) > math.radians(25) or abs(Lleg_position+X[4, 0]) > math.radians(45) or abs(Rleg_position+X[4, 0]) > math.radians(45) or abs(X_ref[2, 0]-X[2, 0]) > 2.5:     # constrain
                
                print("out of constrain",X[0, 0])
                self.set_motor_cmd(motor_number=4,torque=0,kd = 0.01)
                self.set_motor_cmd(motor_number=1,torque=0,kd = 0.01)
                self.set_motor_cmd(motor_number=2,torque=0,kd = 0.005)
                self.set_motor_cmd(motor_number=5,torque=0,kd = 0.005)
                self.ros_manager.send_foc_command(0.0, 0.0)
                self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
                self.adva_running_flag = False
                break

    def disableController(self):
        self.running_flag = False
        self.adva_running_flag = False
        self.ros_manager.send_foc_command(0.0, 0.0)
        if self.lqr_thread is not None:
            self.lqr_thread.join()
            print('lqr_joined')
        if self.advalqr_thread is not None:
            self.advalqr_thread.join()
            print('advlqr_joined')
        
        self.ros_manager.get_logger().info("disable controller")
    
    def releaseUnitree(self):
        self.set_motor_cmd(motor_number=1, kp=0., kd=0, position=0, torque=0, velocity=0)
        self.set_motor_cmd(motor_number=2, kp=0., kd=0, position=0, torque=0, velocity=0)
        self.set_motor_cmd(motor_number=4, kp=0., kd=0, position=0, torque=0, velocity=0)
        self.set_motor_cmd(motor_number=5, kp=0., kd=0, position=0, torque=0, velocity=0)
        self.ros_manager.motor_cmd_pub.publish(self.cmd_list)



def main(args=None):
    robot = robotController()
    command_dict = {
        "start": robot.startController,
        "startadv": robot.startadvaController,
        "d": robot.disableController,
        "r": robot.releaseUnitree,
        "i": robot.init_unitree_motor,
        "l": robot.locklegs,
        "e": robot.enable_ros_manager,
        "stand": robot.standup,
        "p":robot.plotResult,
    }

    while True:
        try:
            cmd = input("CMD :")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                robot.disableController()
                robot.releaseUnitree()
                rclpy.shutdown()
                break
        except KeyboardInterrupt:
            robot.disableController()
            robot.ros_manager_thread.join()
            rclpy.shutdown()
            break
        # except Exception as e:
        #     traceback.print_exc()
        #     break


if __name__ == '__main__':
    main()