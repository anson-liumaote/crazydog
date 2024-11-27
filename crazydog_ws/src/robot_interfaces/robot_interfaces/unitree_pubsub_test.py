import time 

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from unitree_msgs.msg import LowCommand, LowState, MotorCommand, MotorState
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

class UnitreeInterface(Node):

    def __init__(self):
        super().__init__('unitree_pubsub')
        
        self.l_state = LowState()
        self.r_state = LowState()
        
        self.unitree_l_state_sub = self.create_subscription(
            LowState,
            'unitree_l_status',
            self.l_state_callback,
            10,
        )
        self.unitree_r_state_sub = self.create_subscription(
            LowState,
            'unitree_r_status',
            self.r_state_callback,
            10,
        )
        
        # time.sleep(0.1) # Wait for state update
        self.status_pub = self.create_publisher(LowState, 'unitree_status', 1)
        self.pub_timer = self.create_timer(0.004, self.pubTimer_callback)

    def l_state_callback(self, msg: LowState):
        self.l_state = msg

    def r_state_callback(self, msg: LowState):
        self.r_state = msg
    
    def pubTimer_callback(self):
        msg = LowState()
        msg.motor_state[1] = self.l_state.motor_state[1]
        msg.motor_state[2] = self.l_state.motor_state[2]
        msg.motor_state[4] = self.r_state.motor_state[4]
        msg.motor_state[5] = self.r_state.motor_state[5]
        self.status_pub.publish(msg)
    
def main(args=None):
    rclpy.init(args=args)

    unitree_interface = UnitreeInterface()
    
    try:
        rclpy.spin(unitree_interface)
    except KeyboardInterrupt:
        unitree_interface.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()

            
        
    
