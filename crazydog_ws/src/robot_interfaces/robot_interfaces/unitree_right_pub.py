import time
import sys
sys.path.append('/home/crazydog/crazydog/crazydog_ws/src/robot_interfaces/robot_interfaces/unitree_actuator_sdk/lib')
sys.path.append('/home/crazydogv2/crazydog/crazydog_ws/src/robot_interfaces/robot_interfaces/unitree_actuator_sdk/lib')
from unitree_actuator_sdk import * # type: ignorei
import threading
import rclpy
from rclpy.node import Node
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
sys.path.append('..')
# from unitree_left_pub import unitree_communication
from crazydog_ws.src.robot_interfaces.robot_interfaces.unitree_left_pub import unitree_communication

class UnitreeInterface(Node):

    def __init__(self):
        super().__init__('unitree_right_pub')

        ## Declare the top-level 'motors' parameter to read the nested structure
        self.declare_parameter('activate_motors')
        ids = self.get_parameter('activate_motors').value

        self.unitree = unitree_communication('/dev/unitree-r')
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
        self.status_pub = self.create_publisher(LowState, 'unitree_status_right', 1)
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