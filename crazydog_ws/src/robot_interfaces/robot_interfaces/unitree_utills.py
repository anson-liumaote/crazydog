import time
import math
import sys
sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/robot_interfaces/robot_interfaces/unitree_actuator_sdk/lib')
sys.path.append('..')
from unitree_actuator_sdk import * # type: ignorei

class unitree_communication():
    def __init__(self,device_name = '/dev/ttyUSB0'):
        self.serial = SerialPort(device_name)
        self.motors = []
        

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