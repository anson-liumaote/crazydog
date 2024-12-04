import time
import sys
sys.path.append('/home/crazydogv2/crazydog/crazydog_ws/src/robot_interfaces/robot_interfaces/unitree_actuator_sdk/lib')
# sys.path.append('..')
from unitree_actuator_sdk import * # type: ignorei
import threading
import rclpy
from rclpy.node import Node
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState


class unitree_communication(object):
    def __init__(self,device_name = '/dev/ttyUSB0'):
        self.serial = SerialPort(device_name)
        self.motors = []
        # self.runing_flag = False

    def createMotor(self, motor_number=None, MAX=None, MIN=None, origin_pos=None, scale=None):
        if motor_number not in [motor.id for motor in self.motors]:
            motor = unitree_motor(motor_number, MAX_degree=MAX, MIN_degree=MIN, origin_pos=origin_pos, scale=scale)
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
        id = []
        for motor in self.motors:
            if motor.min <= (motor.data.q-motor.origin)/motor.scale <= motor.max:
                self.serial.sendRecv(motor.cmd, motor.data)
                # time.sleep(0.1)
            else:
                print("motor {0} out off constrant".format(motor.cmd.id))
                print(motor.max, motor.data.q, motor.min)
                motor.cmd.mode = queryMotorMode(MotorType.GO_M8010_6,MotorMode.FOC)
                motor.cmd.q    = 0
                motor.cmd.dq   = 0
                motor.cmd.kp   = 0
                motor.cmd.kd   = 0.05
                motor.cmd.tau  = 0
                self.serial.sendRecv(motor.cmd, motor.data)
                success = False
                id.append(motor.cmd.id)
                # time.sleep(0.0006)
        return success, id

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
    def __init__(self, motor_id=None,MAX_degree=None,MIN_degree=None, origin_pos=None, scale=None):
        self.id = motor_id
        self.cmd = MotorCmd()
        self.data = MotorData()
        self.data.motorType = MotorType.GO_M8010_6
        self.cmd.motorType = MotorType.GO_M8010_6
        self.max = MAX_degree
        self.min = MIN_degree
        self.origin = origin_pos
        self.scale = scale
        self.cmd.id = motor_id

        print(f'constrain: id {self.id}, max {self.max}, min {self.min}')


class UnitreeInterface(Node):

    def __init__(self):
        super().__init__('unitree_left_pub')

        ## Declare the top-level 'motors' parameter to read the nested structure
        self.declare_parameter('activate_motors',)
        ids = self.get_parameter('activate_motors').value

        self.unitree = unitree_communication('/dev/unitree-l')
        for id in ids:
            self.declare_parameter(f'motors.{id}.origin')
            self.declare_parameter(f'motors.{id}.scale')
            self.declare_parameter(f'motors.{id}.max')
            self.declare_parameter(f'motors.{id}.min')
            origin = self.get_parameter(f'motors.{id}.origin').value
            scale = self.get_parameter(f'motors.{id}.scale').value
            max = self.get_parameter(f'motors.{id}.max').value
            min = self.get_parameter(f'motors.{id}.min').value
            self.unitree.createMotor(motor_number=id, MAX=max, MIN=min, origin_pos=origin, scale=scale)
        self.unitree_command_sub = self.create_subscription(
            LowCommand,
            'unitree_command',
            self.command_callback,
            1)
        self.status_pub = self.create_publisher(LowState, 'unitree_status_left', 1)
        self.unitree.enableallmotor()
        self.t0 = time.time()
        sendrecv_thread = threading.Thread(target=self.sendrecv_loop)
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

    def sendrecv_loop(self):
        while True:
            feedback, id = self.unitree.motor_sendRecv()
            if feedback==False:
                self.get_logger().error(f'unitree motor {id} out of constrain.')
            msg_list = LowState()
            
            for motor in self.unitree.motors:
                msg = MotorState()
                msg.q = float(motor.data.q)
                msg.dq = float(motor.data.dq)
                msg.temperature = int(motor.data.temp)
                id = motor.id
                msg_list.motor_state[id] = msg
            self.status_pub.publish(msg_list)
    
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