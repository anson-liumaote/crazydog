import time
import math
import rclpy
from rclpy.node import Node
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

MOTOR_INIT_POS = [None, 0.669, 1.1, None, 7.5302, 2.2]

MOTOR_ORIGIN_POS = [0.0, -4.6, 27.8, 0.0, 12.8, -24.4, 0.0, 0.0]
SCALE = [6.33, 6.33, 6.33*1.6, 6.33, -6.33, -6.33*1.6, 1.0, 1.0]
WHEEL_RADIUS = 0.07     # m

class UnitreeInterface(Node):

    def __init__(self):
        super().__init__('jointstate_pub')

        self.unitree_left_sub = self.create_subscription(
            LowState,
            'unitree_status_left',
            self.unitree_left_callback,
            1)
        self.unitree_right_sub = self.create_subscription(
            LowState,
            'unitree_status_right',
            self.unitree_right_callback,
            1)
        self.status_pub = self.create_publisher(LowState, 'unitree_status_all', 1)
        self.pub_timer = self.create_timer(1/300, self.pub_timer_callback)    # period need to be check
        self.unitree_status_left = LowState()
        self.unitree_status_right = LowState()
        self.foc_status = Float32MultiArray()

        self.jointstate_pub = self.create_publisher(JointState, 'jointstate', 1)
        self.foc_status_sub = self.create_subscription(
            Float32MultiArray,
            'foc_msg',
            self.foc_status_callback,
            1)
        self.jointstate_msg = JointState()
        self.jointstate_msg.header.stamp = self.get_clock().now().to_msg()
        self.jointstate_msg.header.frame_id = ""
        self.jointstate_msg.name = ["hip_l", "thigh_l","calf_l","hip_r","thigh_r","calf_r", "wheel_l", "wheel_r"]
        self.jointstate_msg.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.jointstate_msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def foc_status_callback(self, msg: Float32MultiArray):
        self.foc_status = msg

    def unitree_left_callback(self, msg):
        self.unitree_status_left = msg

    def unitree_right_callback(self, msg):
        self.unitree_status_right = msg

    def pub_timer_callback(self):
        try:
            self.jointstate_msg.header.stamp = self.get_clock().now().to_msg()
            msg_list = LowState()
            for id in range(0, 3):
                msg = MotorState()
                msg.q = self.unitree_status_left.motor_state[id].q
                msg.dq = self.unitree_status_left.motor_state[id].dq
                msg.temperature = self.unitree_status_left.motor_state[id].temperature
                msg_list.motor_state[id] = msg
                self.jointstate_msg.position[id] = self.unitree_status_left.motor_state[id].q
                self.jointstate_msg.velocity[id] = self.unitree_status_left.motor_state[id].dq
            for id in range(3, 6):
                msg = MotorState()
                msg.q = self.unitree_status_right.motor_state[id].q
                msg.dq = self.unitree_status_right.motor_state[id].dq
                msg.temperature = self.unitree_status_right.motor_state[id].temperature
                msg_list.motor_state[id] = msg
                self.jointstate_msg.position[id] = self.unitree_status_right.motor_state[id].q
                self.jointstate_msg.velocity[id] = self.unitree_status_right.motor_state[id].dq
        except Exception as e:
            self.get_logger().warning(str(e))

        # try:
        #     if self.foc_status.data[0] == 513.:   # motor left
        #         self.jointstate_msg.position[6] = -self.foc_status.data[1]
        #         self.jointstate_msg.velocity[6] = -self.foc_status.data[2] * (2 * math.pi / 60)
        #     elif self.foc_status.data[0] == 514.: # motor right
        #         self.jointstate_msg.position[7] = self.foc_status.data[1]
        #         self.jointstate_msg.velocity[7] = self.foc_status.data[2] * (2 * math.pi / 60)
        # except Exception as e:
        #     self.get_logger().warning(f'get foc_status error: {e}')
        
        self.status_pub.publish(msg_list)
        self.jointstate_msg = self.scaling(self.jointstate_msg)
        self.jointstate_pub.publish(self.jointstate_msg)
        
    
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