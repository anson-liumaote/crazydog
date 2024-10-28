import time
import math
import sys
sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/robot_interfaces/robot_interfaces/unitree_actuator_sdk/lib')
sys.path.append('..')
# sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control')
from unitree_actuator_sdk import * # type: ignorei
import threading

import rclpy
from rclpy.node import Node
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

MOTOR_INIT_POS = [None, 0.669, 1.080, None, 1.247+2*math.pi, 2.320]
WHEEL_RADIUS = 0.07     # m

class unitree_communication(object):
    def __init__(self,device_name = '/dev/ttyUSB0'):
        self.serial = SerialPort(device_name)
        self.motors = []
        # self.runing_flag = False

    def createMotor(self,motor_number = 0,MAX = 0,MIN = 0,initalposition = 0):
        if motor_number not in [motor.id for motor in self.motors]:
            motor = unitree_motor(motor_number, MAX_degree=MAX, MIN_degree=MIN, inital_position_check=initalposition)
            self.motors.append(motor)
            return motor                              

        else:
            print("Motor {0} already exist".format(motor_number))
            for motor in self.motors:
                if motor.cmd.id == motor_number:
                    return motor

    # def inital_check(self):
    #     if self.runing_flag:
    #         for motor in self.motors:
                
    #             if motor.inital_position_max >= motor.data.q and motor.data.q >= motor.inital_position_min:
    #                 motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
    #                 motor.cmd.q    = motor.data.q
    #                 motor.cmd.dq   = 0
    #                 motor.cmd.kp   = 6.0
    #                 motor.cmd.kd   = 0.1
    #                 motor.cmd.tau  = 0.0
    #                 # motor.inital_position = motor.data.q
    #                 # motor.max_position = motor.inital_position + motor.max
    #                 # motor.min_position = motor.inital_position + motor.min

    #             else:
    #                 print("motor {0} inital motor failed".format(motor.cmd.id))
    #                 motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
    #                 motor.cmd.q    = 0
    #                 motor.cmd.dq   = 0
    #                 motor.cmd.kp   = 0
    #                 motor.cmd.kd   = 0
    #                 motor.cmd.tau  = 0
    #     else:
    #         print("thread didn't start")

    # def disableallmotor(self):
    #     for motor in self.motors:
    #         motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
    #         motor.cmd.q    = 0.0
    #         motor.cmd.dq   = 0
    #         motor.cmd.kp   = 0.0
    #         motor.cmd.kd   = 0.06
    #         motor.cmd.tau  = 0.0
    #     time.sleep(0.01)
    #     self.runing_flag = False

    # def calibrate_all_motor(self):
    #     if self.runing_flag == False:
    #         for motor in self.motors:
    #             motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.CALIBRATE)
    #         for motor in self.motors:
    #             self.serial.sendRecv(motor.cmd, motor.data)
    #             time.sleep(6)  
    #     else:
    #         print("system still runing can't calibrate")

    def position_force_velocity_cmd(self,motor_number=0,torque=0,kp=0,kd=0,position=0,velocity=0):
        for motor in self.motors:
            if motor.id == motor_number:
                motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                motor.cmd.tau = torque
                motor.cmd.kp = kp
                motor.cmd.kd = kd
                motor.cmd.q = position
                motor.cmd.dq = velocity*queryGearRatio(MotorType.GO_M8010_6)
            
    def motor_sendRecv(self):
        for motor in self.motors:
            if motor.max_position>=motor.data.q and motor.data.q>=motor.min_position:
                self.serial.sendRecv(motor.cmd, motor.data)
                # time.sleep(0.1)
            else:
                print("motor {0} out off constrant".format(motor.cmd.id))
                print(motor.max_position, motor.data.q, motor.min_position)
                motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                motor.cmd.q    = 0
                motor.cmd.dq   = 0
                motor.cmd.kp   = 0
                motor.cmd.kd   = 0
                motor.cmd.tau  = 0
                self.serial.sendRecv(motor.cmd, motor.data)
                # time.sleep(0.0006)

    def enableallmotor(self):       
        for motor in self.motors:
            motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
            motor.cmd.q    = 0
            motor.cmd.dq   = 0
            motor.cmd.kp   = 0
            motor.cmd.kd   = 0
            motor.cmd.tau  = 0
            self.serial.sendRecv(motor.cmd, motor.data)
            time.sleep(0.01)
            print(motor.cmd.id,motor.data.q)

    
class unitree_motor(object):                                                                                  
    def __init__(self,motor_id = 0,MAX_degree = 0,MIN_degree = 0, inital_position_check = 0):
        
        self.id = motor_id
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.data.motorType = MotorType.GO_M8010_6
        self.cmd.motorType = MotorType.GO_M8010_6
        # self.inital_position_cheak_point = inital_position_check
        self.inital_position_max = inital_position_check + 1
        self.inital_position_min = inital_position_check - 1
        self.inital_position = inital_position_check
        # self.inital_position = 0
        self.max_position = inital_position_check + MAX_degree
        self.min_position = inital_position_check + MIN_degree
        self.max = MAX_degree
        self.min = MIN_degree
        self.cmd.id = motor_id
        # self.inital_check_success = False


class UnitreeInterface(Node):

    def __init__(self):
        super().__init__('unitree_pubsub')

        self.unitree = unitree_communication('/dev/unitree-l')
        MOTOR1 = self.unitree.createMotor(motor_number = 1,initalposition = MOTOR_INIT_POS[1],MAX=8.475,MIN=-5.364)
        MOTOR2 = self.unitree.createMotor(motor_number = 2,initalposition = MOTOR_INIT_POS[2],MAX=26.801,MIN=-1)
        self.unitree2 = unitree_communication('/dev/unitree-r')
        MOTOR4 = self.unitree2.createMotor(motor_number = 4,initalposition = MOTOR_INIT_POS[4],MAX=5.364,MIN=-8.475)
        MOTOR5 = self.unitree2.createMotor(motor_number = 5,initalposition = MOTOR_INIT_POS[5],MAX=1,MIN=-26.801)

        self.unitree_command_sub = self.create_subscription(
            LowCommand,
            'unitree_command',
            self.command_callback,
            1)
        self.status_pub = self.create_publisher(LowState, 'unitree_status', 1)
        self.unitree.enableallmotor()
        self.unitree2.enableallmotor()
        self.recv_timer = self.create_timer(0.008, self.recv_timer_callback)    # period need to be check

        # self.jointstate_pub = self.create_publisher(JointState, 'jointstate', 1)
        # self.foc_status_sub = self.create_subscription(
        #     Float32MultiArray,
        #     'foc_msg',
        #     self.foc_status_callback,
        #     1)
        # self.jointstate_msg = JointState()
        # self.jointstate_msg.header.stamp = self.get_clock().now().to_msg()
        # self.jointstate_msg.header.frame_id = ""
        # self.jointstate_msg.name = ["hip_l", "thigh_l","calf_l","hip_r","thigh_r","calf_r", "wheel_l", "wheel_r"]
        # self.jointstate_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        # self.jointstate_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def command_callback(self, msg):
        for id, cmd in enumerate(msg.motor_cmd):
            motor_number = id
            torque = cmd.tau
            kp = cmd.kp
            kd = cmd.kd
            position = cmd.q
            velocity = cmd.dq
            self.unitree.position_force_velocity_cmd(motor_number, torque, kp, kd, position, velocity)
            self.unitree2.position_force_velocity_cmd(motor_number, torque, kp, kd, position, velocity)

    # def foc_status_callback(self, msg):
    #     self.jointstate_msg.header.stamp = self.get_clock().now().to_msg()
    #     if msg.data[0] == 513.:   # motor left
    #         self.jointstate_msg.position[6] = -msg.data[1]
    #         self.jointstate_msg.velocity[6] = -msg.data[2] * WHEEL_RADIUS * (2 * math.pi / 60)
    #     elif msg.data[0] == 514.: # motor right
    #         self.jointstate_msg.position[7] = msg.data[1]
    #         self.jointstate_msg.velocity[7] = msg.data[2] * WHEEL_RADIUS * (2 * math.pi / 60)

    def recv_timer_callback(self):
        self.unitree.motor_sendRecv()
        self.unitree2.motor_sendRecv()
        msg_list = LowState()
        # self.jointstate_msg.header.stamp = self.get_clock().now().to_msg()
        
        for motor in self.unitree.motors:
            msg = MotorState()
            msg.q = float(motor.data.q)
            msg.dq = float(motor.data.dq)
            msg.temperature = int(motor.data.temp)
            id = motor.id
            msg_list.motor_state[id] = msg
            # self.jointstate_msg.position[id] = float(motor.data.q)
            # self.jointstate_msg.velocity[id] = float(motor.data.dq)
        for motor in self.unitree2.motors:
            msg = MotorState()
            msg.q = float(motor.data.q)
            msg.dq = float(motor.data.dq)
            msg.temperature = int(motor.data.temp)
            id = motor.id
            msg_list.motor_state[id] = msg
            # self.jointstate_msg.position[id] = float(motor.data.q)
            # self.jointstate_msg.velocity[id] = float(motor.data.dq)
        self.status_pub.publish(msg_list)
        # self.jointstate_pub.publish(self.jointstate_msg)


def main(args=None):
    rclpy.init(args=args)

    unitree_interface = UnitreeInterface()

    rclpy.spin(unitree_interface)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    unitree_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()