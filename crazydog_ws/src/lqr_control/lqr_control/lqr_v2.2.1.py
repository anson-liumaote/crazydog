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

WHEEL_RADIUS = 0.07     # m
WHEEL_MASS = 0.28  # kg
WHEEL_DISTANCE = 0.355
URDF_PATH = "/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control/robot_models/bipedal_new/urdf/bipedal_new.urdf"
MID_ANGLE = 0.0   #0.05
TORQUE_CONSTRAIN = 1.5
MOTOR_INIT_POS = [None, 0.669, 1.080, None, 1.247+2*math.pi, 2.320]     # for unitree motors
INIT_ANGLE = [-2.42, 2.6]
LOCK_POS = [None, 2.76, 9.88, None, 5.44, -3.10]    # -2.75, 2.0
THIGH_LENGTH = 0.215
CALF_LENGTH = 0.215
ORIGIN_BIAS = [-0.02235, 0.22494]   # bias between hip joint and origin in urdf

class robotController():
    def __init__(self) -> None:
        rclpy.init()
        # K: [[ 2.97946709e-07  7.36131891e-05 -1.28508761e+01 -4.14185118e-01]]
        Q = np.diag([0., 10., 100., 0.1])       # 1e-9, 0.1, 1.0, 1e-4
        R = np.diag(np.diag([2.5]))   #0.02 2.5
        q = np.array([0., 0., 0., 0., 0., 0., 1.,
                            0., -1.18, 2.0, 1., 0.,
                            0., -1.18, 2.0, 1., 0.])
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
                                                  slice_w=0.03)
        self.lqr_controller.change_K(self.l_bar)
        # l_bar from 0.1 to 0.37
        self.jump_flag = False


    def enable_ros_manager(self):
        self.ros_manager = RosTopicManager()
        self.ros_manager_thread = threading.Thread(target=rclpy.spin, args=(self.ros_manager,), daemon=True)
        self.ros_manager_thread.start()
        self.running_flag = False
        self.turning_pid = PID(0.5, 0, 0.0001)
        self.cmd_list = LowCommand()
        time.sleep(2)

    def check_target_pos(self, motor_id):
        if motor_id==1 or motor_id==2:
            if self.ros_manager.motor_states[motor_id].q >= MOTOR_INIT_POS[motor_id]:
                return True
            else: 
                return False
        elif motor_id==4 or motor_id==5:
            if self.ros_manager.motor_states[motor_id].q <= MOTOR_INIT_POS[motor_id]:
                return True
            else: 
                return False    
            
    def set_motor_cmd(self, motor_number, kp, kd, position, torque=0, velocity=0):
        cmd = MotorCommand()
        cmd.q = float(position)
        cmd.dq = float(velocity)
        cmd.tau = float(torque)
        cmd.kp = float(kp)
        cmd.kd = float(kd)
        self.cmd_list.motor_cmd[motor_number] = cmd

    def init_unitree_motor(self):
        if self.check_target_pos(1) and self.check_target_pos(4):
            while self.check_target_pos(1) and self.check_target_pos(4):
                # self.set_motor_cmd(motor_number=1, kp=2, kd=0.02, position=self.ros_manager.motor_states[1].q-0.1, torque=0, velocity=0)
                # self.set_motor_cmd(motor_number=4, kp=2, kd=0.02, position=self.ros_manager.motor_states[4].q+0.1, torque=0, velocity=0)
                self.set_motor_cmd(motor_number=1, kp=0, kd=0.2, position=0, torque=0, velocity=-0.5)
                self.set_motor_cmd(motor_number=4, kp=0, kd=0.2, position=0, torque=0, velocity=0.5)
                self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
                time.sleep(0.001)
                
            # time.sleep(0.1)
            self.set_motor_cmd(motor_number=1, kp=6., kd=0.1, position=self.ros_manager.motor_states[1].q, torque=0, velocity=0)
            self.set_motor_cmd(motor_number=2, kp=6., kd=0.1, position=self.ros_manager.motor_states[2].q, torque=0, velocity=0)
            self.set_motor_cmd(motor_number=4, kp=6., kd=0.1, position=self.ros_manager.motor_states[4].q, torque=0, velocity=0)
            self.set_motor_cmd(motor_number=5, kp=6., kd=0.1, position=self.ros_manager.motor_states[5].q, torque=0, velocity=0)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            # unitree.inital_check()
            # unitree2.inital_check()
        else:
            print("inital fail")

    
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
                                0., theta1+1.57, -theta2, 1., 0.])
        self.com, self.l_bar = self.robot.calculateCom()
        theta1_err = theta1 - INIT_ANGLE[0]
        theta2_err = theta2 - INIT_ANGLE[1]
        return theta1_err, theta2_err

    # def locklegs(self):
    #     self.ros_manager.wheel_coordinate = [0.033-0.0752, -0.2285]
    #     theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)     # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
    #     self.lqr_controller.change_K(self.l_bar)
    #     while self.ros_manager.motor_states[1].q <= MOTOR_INIT_POS[1]-theta1_err*6.33 and self.ros_manager.motor_states[4].q >= MOTOR_INIT_POS[4]+theta1_err*6.33:            
    #         # self.set_motor_cmd(motor_number=1, kp=2, kd=0.02, position=self.ros_manager.motor_states[1].q+0.1, torque=0, velocity=0)
    #         # self.set_motor_cmd(motor_number=4, kp=2, kd=0.02, position=self.ros_manager.motor_states[4].q-0.1, torque=0, velocity=0)
    #         self.set_motor_cmd(motor_number=1, kp=0, kd=0.05, position=0, torque=0, velocity=0.2)
    #         self.set_motor_cmd(motor_number=4, kp=0, kd=0.05, position=0, torque=0, velocity=-0.2)
    #         self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
    #         time.sleep(0.001)
    #     for i in range(10):                        
    #         self.set_motor_cmd(motor_number=1, kp=i, kd=0.12, position=MOTOR_INIT_POS[1]-theta1_err*6.33)
    #         self.set_motor_cmd(motor_number=4, kp=i, kd=0.12, position=MOTOR_INIT_POS[4]+theta1_err*6.33)
    #         self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
    #         time.sleep(0.01)
    #     while self.ros_manager.motor_states[2].q <= MOTOR_INIT_POS[2]-theta2_err*6.33*1.6 and self.ros_manager.motor_states[5].q >= MOTOR_INIT_POS[5]+theta2_err*6.33*1.6:
    #         # self.set_motor_cmd(motor_number=2, kp=25, kd=0.02, position=self.ros_manager.motor_states[2].q+0.05, torque=0, velocity=0)
    #         # self.set_motor_cmd(motor_number=5, kp=25, kd=0.02, position=self.ros_manager.motor_states[5].q-0.05, torque=0, velocity=0)
    #         self.set_motor_cmd(motor_number=2, kp=0, kd=0, position=0, torque=0.6, velocity=0)
    #         self.set_motor_cmd(motor_number=5 ,kp=0, kd=0, position=0, torque=-0.6, velocity=0)
    #         self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
    #         time.sleep(0.001)
    #     for i in range(10):                        
    #         self.set_motor_cmd(motor_number=2, kp=i, kd=0.15, position=MOTOR_INIT_POS[2]-theta2_err*6.33*1.6)
    #         self.set_motor_cmd(motor_number=5, kp=i, kd=0.15, position=MOTOR_INIT_POS[5]+theta2_err*6.33*1.6)
    #         self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
    #         time.sleep(0.01)
    
    def standup(self):
        self.ros_manager.wheel_coordinate = [-0.06167-ORIGIN_BIAS[0], 0.1206043-ORIGIN_BIAS[1]]
        theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)     # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
        self.lqr_controller.change_K(self.l_bar)
        self.startController()
        time.sleep(0.5)
        self.set_motor_cmd(motor_number=1, kp=5, kd=0.12, position=MOTOR_INIT_POS[1]-theta1_err*6.33)
        self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
        time.sleep(0.1)
        self.set_motor_cmd(motor_number=4, kp=5, kd=0.12, position=MOTOR_INIT_POS[4]+theta1_err*6.33)
        self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
        time.sleep(0.1)
        self.set_motor_cmd(motor_number=2, kp=10, kd=0.12, position=MOTOR_INIT_POS[2]-theta2_err*6.33*1.6)
        self.set_motor_cmd(motor_number=5, kp=10, kd=0.12, position=MOTOR_INIT_POS[5]+theta2_err*6.33*1.6)
        self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
        

    def update_pose(self):
        theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)
        self.set_motor_cmd(motor_number=1, kp=10, kd=0.12, position=MOTOR_INIT_POS[1]-theta1_err*6.33)
        self.set_motor_cmd(motor_number=4, kp=10, kd=0.12, position=MOTOR_INIT_POS[4]+theta1_err*6.33)
        self.set_motor_cmd(motor_number=2, kp=10, kd=0.12, position=MOTOR_INIT_POS[2]-theta2_err*6.33*1.6)
        self.set_motor_cmd(motor_number=5, kp=10, kd=0.12, position=MOTOR_INIT_POS[5]+theta2_err*6.33*1.6)
        # print(theta1_err, theta2_err)
        self.ros_manager.motor_cmd_pub.publish(self.cmd_list)

    def jump(self):
        for i in range(70):
            self.ros_manager.wheel_coordinate[1] -= 0.001
            self.ros_manager.wheel_coordinate[0] -= 0.0001
            theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)     # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
            self.lqr_controller.change_K(self.l_bar)
            self.set_motor_cmd(motor_number=1, kp=3, kd=0., position=MOTOR_INIT_POS[1]-theta1_err*6.33)
            self.set_motor_cmd(motor_number=4, kp=3, kd=0., position=MOTOR_INIT_POS[4]+theta1_err*6.33)
            self.set_motor_cmd(motor_number=2, kp=3, kd=0., position=MOTOR_INIT_POS[2]-theta2_err*6.33*1.6)
            self.set_motor_cmd(motor_number=5, kp=3, kd=0., position=MOTOR_INIT_POS[5]+theta2_err*6.33*1.6)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.001)
        # time.sleep(0.1)
        # for i in range(100):
        #     self.ros_manager.wheel_coordinate[1] += 0.0005
        #     self.ros_manager.wheel_coordinate[0] += 0.001
        #     theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)     # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
        #     self.lqr_controller.change_K(self.l_bar)
        #     self.set_motor_cmd(motor_number=1, kp=3, kd=0.12, position=MOTOR_INIT_POS[1]-theta1_err*6.33)
        #     self.set_motor_cmd(motor_number=4, kp=3, kd=0.12, position=MOTOR_INIT_POS[4]+theta1_err*6.33)
        #     self.set_motor_cmd(motor_number=2, kp=3, kd=0.12, position=MOTOR_INIT_POS[2]-theta2_err*6.33*1.6)
        #     self.set_motor_cmd(motor_number=5, kp=3, kd=0.12, position=MOTOR_INIT_POS[5]+theta2_err*6.33*1.6)
        #     self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
        #     time.sleep(0.001)
        time.sleep(0.15)
        self.jump_flag = False

    def jump_trigger(self):
        self.jump_flag = True

    def calibrate_com(self):
        oMi = self.robot.getOmi()
        err = oMi - self.com[0]
        self.ros_manager.wheel_coordinate[0] -= err*0.5
        print(self.ros_manager.wheel_coordinate, err)

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
            with self.ros_manager.ctrl_condition:
                self.ros_manager.ctrl_condition.wait()
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
            # print(X[0, 0])
            # get IMU data
            X[2, 0], X[3, 0] = self.ros_manager.get_orientation()
            # X[2, 0], _ = self.ros_manager.get_orientation()
            # X[3, 0] = (X[2, 0] - X_last[2, 0]) / dt
            if abs(X[2, 0]) > math.radians(25):     # constrain
                # U[0, 0] = 0.0
                self.ros_manager.send_foc_command(0.0, 0.0)
                continue
            
            # get u from lqr
            U = np.copy(self.lqr_controller.lqr_control(X, X_ref))
            yaw_speed = self.get_yaw_speed(foc_status_left.speed, foc_status_right.speed)
            yaw_torque = self.turning_pid.update(yaw_ref, yaw_speed, dt)

            if (time.time()-start_time) < 3:
                yaw_torque = 0.0
            if (time.time()-start_time) > 3:
                # self.calibrate_com()
                self.update_pose()
                self.lqr_controller.change_K(self.l_bar)
                if self.jump_flag:
                    self.jump()
            motor_command_left = U[0, 0] + yaw_torque
            motor_command_right = U[0, 0] - yaw_torque
            # soft constrain
            motor_command_left = max(-1.5, min(motor_command_left, 1.5))
            motor_command_right = max(-1.5, min(motor_command_right, 1.5))

            self.ros_manager.send_foc_command(motor_command_left, motor_command_right)

            

            if len(X_list)<=1001:
                # print(len(X_list))
                X_list.append(np.copy(X))
                U_list.append(np.copy(U))
                if len(X_list)==1000:
                    formated_time = datetime.now().strftime("%Y_%m_%d_%H_%M_%S")
                    directory = f"/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control/log/{formated_time}"
                    os.makedirs(directory, exist_ok=True)  
                    with open(os.path.join(directory, 'x.plk'), 'wb') as f1:
                        pickle.dump(X_list, f1)
                    with open(os.path.join(directory, 'u.plk'), 'wb') as f2:
                        pickle.dump(U_list, f2)
            # else:
            #     print('log saved')


    def disableController(self):
        self.running_flag = False
        self.ros_manager.send_foc_command(0.0, 0.0)
        if self.lqr_thread is not None:
            self.lqr_thread.join()
            print('lqr_joined')
        
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
        "d": robot.disableController,
        "r": robot.releaseUnitree,
        "i": robot.init_unitree_motor,
        # "l": robot.locklegs,
        "e": robot.enable_ros_manager,
        "stand": robot.standup,
        "j" : robot.jump_trigger,
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