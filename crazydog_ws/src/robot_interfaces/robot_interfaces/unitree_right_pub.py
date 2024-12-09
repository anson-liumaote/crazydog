import time
import math
from .unitree_utills import unitree_communication

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

MOTOR_INIT_POS = [None, 0.478, 1.190, None, 1.247+2*math.pi, 0.944]
WHEEL_RADIUS = 0.07     # m

class UnitreeRInterface(Node):

    def __init__(self):
        super().__init__('unitree_r_pubsub')

        self.unitree = unitree_communication('/dev/unitree-r')
        MOTOR4 = self.unitree.createMotor(motor_number = 4,initalposition = MOTOR_INIT_POS[4],MAX=5.364,MIN=-8.475)
        MOTOR5 = self.unitree.createMotor(motor_number = 5,initalposition = MOTOR_INIT_POS[5],MAX=1,MIN=-26.801)

        self.cb_group = ReentrantCallbackGroup()
        self.r_status_pub = self.create_publisher(LowState, 'unitree_r_status', 10)
        self.unitree.enableallmotor()
        self.unitree_r_timer = self.create_timer(0.001, self.unitree_r_callback, callback_group=self.cb_group)

        self.unitree_command_sub = self.create_subscription(
            LowCommand,
            'unitree_command',
            self.command_callback,
            1,
            callback_group=self.cb_group)
    
    def unitree_r_callback(self):
        msg_list = LowState()
        self.unitree.motor_sendRecv()
        for motor in self.unitree.motors:
            msg = MotorState()
            msg.q = float(motor.data.q)
            msg.dq = float(motor.data.dq)
            msg.tau = float(motor.data.tau)
            msg.temperature = int(motor.data.temp)
            id = motor.id
            msg_list.motor_state[id] = msg
        self.r_status_pub.publish(msg_list)   

    def command_callback(self, msg):
        for id, cmd in enumerate(msg.motor_cmd):
            motor_number = id
            torque = cmd.tau
            kp = cmd.kp
            kd = cmd.kd
            position = cmd.q
            velocity = cmd.dq
            self.unitree.position_force_velocity_cmd(motor_number, torque, kp, kd, position, velocity)
    
def main(args=None):
    rclpy.init(args=args)

    unitree_interface_r = UnitreeRInterface()
    
    try:
        rclpy.spin(unitree_interface_r)
    except KeyboardInterrupt:
        unitree_interface_r.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
