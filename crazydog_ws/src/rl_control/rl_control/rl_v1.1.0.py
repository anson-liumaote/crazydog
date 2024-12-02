import rclpy
import threading
import time
import math
import numpy as np
from utils.pid import PID
from utils.ros_manager import RosManager
# from utils.motor_getready import disableUnitreeMotor, init_unitree_motor, locklegs, enable
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
import pickle
from datetime import datetime
import os
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import onnxruntime as ort
from scipy.spatial.transform import Rotation as R

WHEEL_RADIUS = 0.07     # m
WHEEL_MASS = 0.28  # kg
WHEEL_DISTANCE = 0.355
TORQUE_CONSTRAIN = 1.5
MOTOR_INIT_POS = [None, 0.669, 1.080, None, 7.5302, 2.320]     # for unitree motors
INIT_ANGLE = [-2.42, 2.6]
LOCK_POS = [None, 2.76, 9.88, None, 5.44, -3.10]    # -2.75, 2.0
THIGH_LENGTH = 0.215
CALF_LENGTH = 0.215
ORIGIN_BIAS = [-0.02235, 0.22494]   # bias between hip joint and origin in urdf
MOTOR_ORIGIN_POS = [0.0, -4.6, 27.8, 0.0, 12.8, -24.4, 0.0, 0.0]
SCALE = [6.33, 6.33, 6.33*1.6, 6.33, -6.33, -6.33*1.6, 1.0, 1.0]

class robotController():
    def __init__(self) -> None:
        rclpy.init()
        self.rl_thread = None
        # Initialize ONNX model
        self.model_path = '/home/crazydog/crazydog/crazydog_ws/src/rl_control/rl_control/model/2024-11-24_23-20-58/exported/policy.onnx'
        self.ort_session = ort.InferenceSession(self.model_path)


    def enable_ros_manager(self):
        self.ros_manager = RosManager()
        self.ros_manager_thread = threading.Thread(target=rclpy.spin, args=(self.ros_manager,), daemon=True)
        self.ros_manager_thread.start()
        self.running_flag = False
        self.wheel_pid_l = PID(0.3, 0.0, 0.0)
        self.wheel_pid_r = PID(0.3, 0.0, 0.0)
        self.cmd_list = LowCommand()
        time.sleep(2)

    def check_target_pos(self, j_name):
        if self.ros_manager.get_joint_pos(j_name) >= 0.8324:
            return True
        else: 
            return False
    
    # def scaling(self, id, pos, vel):
    #     states.position = (os-org)/scale 
    #     states.velocity = [state/scale for state, scale in zip(states.velocity, self.scale)]
    #     return states

    def inverse_scaling(self, id, pos, vel):
        position = pos * SCALE[id] + MOTOR_ORIGIN_POS[id]
        velocity = vel * SCALE[id]
        return float(position), float(velocity)
            
    def set_motor_cmd(self, motor_number, kp, kd, position, torque=0, velocity=0, scaling=False):
        if scaling==False:
            torque = max(-1, min(torque, 1))
            cmd = MotorCommand()
            cmd.q = float(position)
            cmd.dq = float(velocity)
            # cmd.q, cmd.dq = self.inverse_scaling(position, velocity)
            cmd.tau = float(torque)
            cmd.kp = float(kp)
            cmd.kd = float(kd)
            self.cmd_list.motor_cmd[motor_number] = cmd
        else:
            torque = max(-1, min(torque, 1))
            cmd = MotorCommand()
            cmd.q = float(position) * SCALE[motor_number] + MOTOR_ORIGIN_POS[motor_number]
            cmd.dq = float(velocity) * SCALE[motor_number]
            # cmd.q, cmd.dq = self.inverse_scaling(position, velocity)
            cmd.tau = float(torque)/SCALE[motor_number]
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
        theta1_err = theta1 - INIT_ANGLE[0]
        theta2_err = theta2 - INIT_ANGLE[1]
        return theta1_err, theta2_err

    def locklegs(self):
        # self.ros_manager.wheel_coordinate = [-0.0639-ORIGIN_BIAS[0], -0.003637-ORIGIN_BIAS[1]]
        # theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)     # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
        while self.ros_manager.get_joint_pos('thigh_l') <= 1.271 and self.ros_manager.get_joint_pos('thigh_r') <= 1.271:
            self.set_motor_cmd(motor_number=1, kp=0, kd=0.05, position=0, torque=0, velocity=0.2, scaling=False)
            self.set_motor_cmd(motor_number=4, kp=0, kd=0.05, position=0, torque=0, velocity=-0.2, scaling=False)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.001)
        for i in range(10):                        
            self.set_motor_cmd(motor_number=1, kp=i, kd=0.12, position=1.271, scaling=True)
            self.set_motor_cmd(motor_number=4, kp=i, kd=0.12, position=1.271, scaling=True)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)
        while self.ros_manager.get_joint_pos('calf_l') <= -2.12773 and self.ros_manager.get_joint_pos('calf_r') <= -2.12773:
            self.set_motor_cmd(motor_number=2, kp=0, kd=0, position=0, torque=0.7, velocity=0, scaling=False)
            self.set_motor_cmd(motor_number=5 ,kp=0, kd=0, position=0, torque=-0.7, velocity=0, scaling=False)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.001)
        for i in range(10):                        
            self.set_motor_cmd(motor_number=2, kp=i, kd=0.15, position=-2.12773, scaling=True)
            self.set_motor_cmd(motor_number=5, kp=i, kd=0.15, position=-2.12773, scaling=True)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)        

    def startController(self):
        self.prev_pitch = 0
        self.rl_thread = threading.Thread(target=self.controller)
        self.running_flag = True
        self.rl_thread.start()

    def controller(self):
        self.ros_manager.get_logger().info('controller start')
        actions = np.zeros(6)  # Initial actions
        count = 0
        t0 = time.time()

        while self.running_flag:
            with self.ros_manager.ctrl_condition:
                self.ros_manager.ctrl_condition.wait()
            # Prepare the input as a single (22,) array
            input_data = np.concatenate([
                self.ros_manager.base_lin_vel,
                self.ros_manager.base_ang_vel,
                self.ros_manager.projected_gravity,
                self.ros_manager.velocity_commands,
                self.ros_manager.joint_pos,
                self.ros_manager.joint_vel,
                actions
            ]).astype(np.float32).reshape(1, -1)  # Model expects (1, 24)

            # Run inference
            print(input_data)
            input_name = self.ort_session.get_inputs()[0].name
            outputs = self.ort_session.run(None, {input_name: input_data})
            
            # Process output (e.g., actions) as needed for control
            actions = outputs[0].flatten()  # Assuming model output is (1, 4) for actions

            # Optionally publish or use actions to control the robot
            print(actions)
            # self.set_motor_cmd(motor_number=1, kp=0, kd=0, position=0, torque=actions[0]/6.33, velocity=0)
            # self.set_motor_cmd(motor_number=2 ,kp=0, kd=0, position=0, torque=actions[2]/6.33/1.6, velocity=0)
            # self.set_motor_cmd(motor_number=4, kp=0, kd=0, position=0, torque=-actions[1]/6.33, velocity=0)
            # self.set_motor_cmd(motor_number=5 ,kp=0, kd=0, position=0, torque=-actions[3]/6.33/1.6, velocity=0)

            self.set_motor_cmd(motor_number=1, kp=25.0, kd=0.5, position=actions[0]+1.271, torque=0, velocity=0, scaling=True)
            self.set_motor_cmd(motor_number=2, kp=25.0, kd=0.5, position=actions[2]+1.271, torque=0, velocity=0, scaling=True)
            self.set_motor_cmd(motor_number=4, kp=25.0, kd=0.5, position=actions[1]+(-2.12773), torque=0, velocity=0, scaling=True)
            self.set_motor_cmd(motor_number=5, kp=25.0, kd=0.5, position=actions[3]+(-2.12773), torque=0, velocity=0, scaling=True)
            # print(self.cmd_list)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)

            t1 = time.time()
            dt = t1-t0
            print(f'freq, {1/dt}')
            t0 = t1
            wheel_tau_l = self.wheel_pid_l.update(actions[4], self.ros_manager.joint_vel[2], 0.02)
            wheel_tau_r = self.wheel_pid_r.update(actions[5], self.ros_manager.joint_vel[5], 0.02)
            print(self.ros_manager.joint_vel[2], self.ros_manager.joint_vel[5])
            print(wheel_tau_l, wheel_tau_r)
            self.ros_manager.send_foc_command(wheel_tau_l, wheel_tau_r)
            count += 1


    def disableController(self):
        self.running_flag = False
        self.ros_manager.send_foc_command(0.0, 0.0)
        self.releaseUnitree()
        if self.rl_thread is not None:
            self.rl_thread.join()
            print('rl_joined')
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
        "l": robot.locklegs,
        "e": robot.enable_ros_manager,
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