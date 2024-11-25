import time
import math
import sys
sys.path.append('/home/crazydogcore/crazydog/crazydog_ws/src/robot_interfaces/robot_interfaces/unitree_actuator_sdk/lib')
sys.path.append('..')
# sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/lqr_control/lqr_control')
from unitree_actuator_sdk import * # type: ignorei
import threading

import rclpy
from rclpy.node import Node
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

MOTOR_INIT_POS = [None, 0.669, 1.1, None, 7.5302, 2.2]

MOTOR_ORIGIN_POS = [0.0, -4.6, 27.8, 0.0, 12.8, -24.4, 0.0, 0.0]
SCALE = [6.33, 6.33, 6.33*1.6, 6.33, -6.33, -6.33*1.6, 1.0, 1.0]
WHEEL_RADIUS = 0.07     # m

class unitree_communication(object):
    def __init__(self,device_name = '/dev/ttyUSB0'):
        self.serial = SerialPort(device_name)
        self.motors = []
        # self.runing_flag = False

    def createMotor(self, motor_number=None, MAX=None, MIN=None):
        if motor_number not in [motor.id for motor in self.motors]:
            motor = unitree_motor(motor_number, MAX_degree=MAX, MIN_degree=MIN)
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
        success = True
        for motor in self.motors:
            if motor.min <= motor.data.q <= motor.max:
                self.serial.sendRecv(motor.cmd, motor.data)
                # time.sleep(0.1)
            else:
                print("motor {0} out off constrant".format(motor.cmd.id))
                print(motor.max, motor.data.q, motor.min)
                motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                motor.cmd.q    = 0
                motor.cmd.dq   = 0
                motor.cmd.kp   = 0
                motor.cmd.kd   = 0
                motor.cmd.tau  = 0
                self.serial.sendRecv(motor.cmd, motor.data)
                success = False
                # time.sleep(0.0006)
        return success

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
    def __init__(self, motor_id=None,MAX_degree=None,MIN_degree=None):
        self.id = motor_id
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.data.motorType = MotorType.GO_M8010_6
        self.cmd.motorType = MotorType.GO_M8010_6
        self.max = MAX_degree * SCALE[motor_id] + MOTOR_ORIGIN_POS[motor_id]
        self.min = MIN_degree * SCALE[motor_id] + MOTOR_ORIGIN_POS[motor_id]
        self.cmd.id = motor_id

        print(f'constrain: id {self.id}, max {self.max}, min {self.min}')


class UnitreeInterface(Node):

    def __init__(self):
        super().__init__('unitree_left_pub')

        self.unitree = unitree_communication('/dev/unitree-l')
        MOTOR1 = self.unitree.createMotor(motor_number = 1, MAX=2.093, MIN=0.0)
        MOTOR2 = self.unitree.createMotor(motor_number = 2, MAX=0.0, MIN=-2.7)
        self.unitree_command_sub = self.create_subscription(
            LowCommand,
            'unitree_command',
            self.command_callback,
            1)
        self.status_pub = self.create_publisher(LowState, 'unitree_status_left', 1)
        self.unitree.enableallmotor()
        # self.recv_timer = self.create_timer(0.008, self.recv_timer_callback)    # period need to be check
        self.t0 = time.time()
        sendrecv_thread = threading.Thread(target=self.recv_timer_callback)
        sendrecv_thread.start()

    def command_callback(self, msg:LowCommand):
        for id, cmd in enumerate(msg.motor_cmd):
            motor_number = id
            torque = cmd.tau
            kp = cmd.kp
            kd = cmd.kd
            position = cmd.q
            velocity = cmd.dq
            self.unitree.position_force_velocity_cmd(motor_number, torque, kp, kd, position, velocity)

    def recv_timer_callback(self):
        while True:
            feedback = self.unitree.motor_sendRecv()
            if feedback==False:
                self.get_logger().error('unitree motor out of constrain.')
            msg_list = LowState()
            
            for motor in self.unitree.motors:
                msg = MotorState()
                msg.q = float(motor.data.q)
                msg.dq = float(motor.data.dq)
                msg.temperature = int(motor.data.temp)
                id = motor.id
                msg_list.motor_state[id] = msg
            self.status_pub.publish(msg_list)
    
    def scaling(self, states: JointState):
        states.position = [(state-org)/scale for state, scale, org in zip(states.position, SCALE, MOTOR_ORIGIN_POS)]
        states.velocity = [state/scale for state, scale in zip(states.velocity, SCALE)]
        return states
    
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