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
from scipy.spatial.transform import Rotation as R
import onnxruntime as ort

MOTOR_ORIGIN_POS = [0.0, -4.5, 27.8, 0.0, 14.8, -25.6, 0.0, 0.0]
SCALE = [6.33, 6.33, 6.33*1.6, 6.33, -6.33, -6.33*1.6, 1.0, 1.0]

class robotController():
    def __init__(self) -> None:
        rclpy.init()
        self.rl_thread = None
        # Initialize ONNX model
        self.model_path = '/home/crazydog/crazydog/crazydog_ws/src/rl_control/rl_control/model/2024-12-10_22-19-38/exported/policy.onnx'
        self.ort_session = ort.InferenceSession(self.model_path)


    def enable_ros_manager(self):
        self.ros_manager = RosManager()
        self.ros_manager_thread = threading.Thread(target=rclpy.spin, args=(self.ros_manager,), daemon=True)
        self.ros_manager_thread.start()
        self.running_flag = False
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

    def locklegs(self):
        # self.ros_manager.wheel_coordinate = [-0.0639-ORIGIN_BIAS[0], -0.003637-ORIGIN_BIAS[1]]
        # theta1_err, theta2_err = self.get_angle_error(self.ros_manager.wheel_coordinate)     # lock legs coordinate [x, y] (hip joint coordinate (0.0742, 0))
        while self.ros_manager.get_joint_pos('thigh_l') <= 1.271 and self.ros_manager.get_joint_pos('thigh_r') <= 1.271:
            self.set_motor_cmd(motor_number=1, kp=0, kd=0.05, position=0, torque=0, velocity=0.2, scaling=False)
            self.set_motor_cmd(motor_number=4, kp=0, kd=0.05, position=0, torque=0, velocity=-0.2, scaling=False)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.001)
        for i in range(20):                        
            self.set_motor_cmd(motor_number=1, kp=i, kd=0.12, position=1.271, scaling=True)
            self.set_motor_cmd(motor_number=4, kp=i, kd=0.12, position=1.271, scaling=True)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)
        while self.ros_manager.get_joint_pos('calf_l') <= -2.12773 and self.ros_manager.get_joint_pos('calf_r') <= -2.12773:
            self.set_motor_cmd(motor_number=2, kp=0, kd=0, position=0, torque=0.7, velocity=0, scaling=False)
            self.set_motor_cmd(motor_number=5 ,kp=0, kd=0, position=0, torque=-0.7, velocity=0, scaling=False)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.001)
        for i in range(20):                        
            self.set_motor_cmd(motor_number=2, kp=i, kd=0.15, position=-2.12773, scaling=True)
            self.set_motor_cmd(motor_number=5, kp=i, kd=0.15, position=-2.12773, scaling=True)
            self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            time.sleep(0.01)        

    def startController(self):
        self.prev_pitch = 0
        self.rl_thread = threading.Thread(target=self.controller)
        self.running_flag = True
        self.rl_thread.start()

    def pd_controller(self, kp, tq, q, kd, td, d):
        return kp * (tq - q) + kd * (td - d)

    def controller(self):
        self.ros_manager.get_logger().info('controller start')
        actions = np.zeros(2)  # Initial actions
        obs_list = []
        action_list = []
        output_list = []
        time_list = []

        while self.running_flag:
            with self.ros_manager.ctrl_condition:
                self.ros_manager.ctrl_condition.wait()
            # Prepare the input as a single (22,) array
            input_data = np.concatenate([
                self.ros_manager.base_lin_vel,
                self.ros_manager.base_ang_vel,
                self.ros_manager.projected_gravity,
                self.ros_manager.velocity_commands,
                # self.ros_manager.joint_pos,
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
            # tau1 = self.pd_controller(25.0, actions[0], self.ros_manager.joint_pos[0], 0.5, 0.0, self.ros_manager.joint_vel[0])
            # tau2 = self.pd_controller(25.0, actions[2], self.ros_manager.joint_pos[2], 0.5, 0.0, self.ros_manager.joint_vel[1])
            # tau4 = self.pd_controller(25.0, actions[1], self.ros_manager.joint_pos[1], 0.5, 0.0, self.ros_manager.joint_vel[3])
            # tau5 = self.pd_controller(25.0, actions[3], self.ros_manager.joint_pos[3], 0.5, 0.0, self.ros_manager.joint_vel[4])
            wheel_tau_l = self.pd_controller(0.0, 0.0, 0.0, 0.3, actions[0], self.ros_manager.joint_vel[0])
            wheel_tau_r = self.pd_controller(0.0, 0.0, 0.0, 0.3, actions[1], self.ros_manager.joint_vel[1])

            # self.set_motor_cmd(motor_number=1, kp=0, kd=0, position=0, torque=tau1, velocity=0, scaling=True)
            # self.set_motor_cmd(motor_number=2, kp=0, kd=0, position=0, torque=tau2, velocity=0, scaling=True)
            # self.set_motor_cmd(motor_number=4, kp=0, kd=0, position=0, torque=tau4, velocity=0, scaling=True)
            # self.set_motor_cmd(motor_number=5, kp=0, kd=0, position=0, torque=tau5, velocity=0, scaling=True)
            # self.ros_manager.motor_cmd_pub.publish(self.cmd_list)
            # print(tau1, tau2, tau4, tau5, wheel_tau_l, wheel_tau_r)
            print(wheel_tau_l, wheel_tau_r)

            self.ros_manager.send_foc_command(wheel_tau_l, wheel_tau_r)
            
            obs_list.append(input_data)
            action_list.append(actions)
            # output_list.append(np.array([tau1, tau2, tau4, tau5, wheel_tau_l, wheel_tau_r]))
            output_list.append(np.array([wheel_tau_l, wheel_tau_r]))
            time_list.append(np.array(time.time()))

            if len(time_list)==50:  
                formated_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
                directory = f"/home/crazydog/crazydog/crazydog_ws/src/rl_control/rl_control/log/{formated_time}"
                os.makedirs(directory, exist_ok=True)  
                with open(os.path.join(directory, 'obs.plk'), 'wb') as f1:
                    pickle.dump(obs_list, f1)
                with open(os.path.join(directory, 'action.plk'), 'wb') as f2:
                    pickle.dump(action_list, f2)
                with open(os.path.join(directory, 'output.plk'), 'wb') as f3:
                    pickle.dump(output_list, f3)
                with open(os.path.join(directory, 'time.plk'), 'wb') as f4:
                    pickle.dump(time_list, f4)


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