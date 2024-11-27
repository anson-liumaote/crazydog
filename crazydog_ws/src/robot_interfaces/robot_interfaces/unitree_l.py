import time
import math
from .unitree_utills import unitree_communication

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

MOTOR_INIT_POS = [None, 0.669, 1.080, None, 1.247+2*math.pi, 2.320]
WHEEL_RADIUS = 0.07     # m

class UnitreeLInterface(Node):

    def __init__(self):
        super().__init__('unitree_l_pubsub')

        self.unitree = unitree_communication('/dev/unitree-l')
        MOTOR1 = self.unitree.createMotor(motor_number = 1,initalposition = MOTOR_INIT_POS[1],MAX=8.475,MIN=-5.364)
        MOTOR2 = self.unitree.createMotor(motor_number = 2,initalposition = MOTOR_INIT_POS[2],MAX=26.801,MIN=-1)

        self.cb_group = ReentrantCallbackGroup()
        self.l_status_pub = self.create_publisher(LowState, 'unitree_l_status', 10)
        self.unitree.enableallmotor()
        self.unitree_l_timer = self.create_timer(0.001, self.unitree_l_callback, callback_group=self.cb_group)

        self.unitree_command_sub = self.create_subscription(
            LowCommand,
            'unitree_command',
            self.command_callback,
            1,
            callback_group=self.cb_group)


    def unitree_l_callback(self):
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
        self.l_status_pub.publish(msg_list)
        
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

    unitree_interface_l = UnitreeLInterface()
    
    try:
        rclpy.spin(unitree_interface_l)
    except KeyboardInterrupt:
        unitree_interface_l.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()