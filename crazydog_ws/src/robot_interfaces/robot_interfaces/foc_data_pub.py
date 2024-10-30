import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import can
import math
import numpy as np

GEAR_RATIO_M3508 = 15.76
GEAR_RATIO_M2006 = 36.0

class DjiMotor():
    def __init__(self, motor_id, gear_ratio):
        self.id = motor_id
        self.q = None
        self.dq = None
        self.current = None
        self.temp = 0.0
        self.abs_q = None
        self.count = 0
        self.gear_ratio = gear_ratio
    
    def get_states(self):
        return [float(self.id), self.abs_q, self.dq, self.current, self.temp]

class focDataPublisher(Node):

    def __init__(self):
        super().__init__('foc_data_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'foc_msg', 1)
        can_interface = 'can0'
        self.bus = can.interface.Bus(channel=can_interface, interface='socketcan')
        m_left = DjiMotor(motor_id=0x201, gear_ratio=GEAR_RATIO_M3508)
        m_right = DjiMotor(motor_id=0x202, gear_ratio=GEAR_RATIO_M3508)
        m_dock = DjiMotor(motor_id=0x203, gear_ratio=GEAR_RATIO_M2006)
        self.motors = [m_left, m_right, m_dock]
        self.receive_can_messages()
        
    def receive_can_messages(self):
        msg = Float32MultiArray()
        while True:
            try:
                message = self.bus.recv(timeout=1)
                for motor in self.motors:
                    if motor.id == message.arbitration_id:
                        angle = float(((message.data[0] << 8) | message.data[1])/8192*2*math.pi)
                        motor.dq = float(self.twos_complement_16bit((message.data[2] << 8) | message.data[3]))/motor.gear_ratio
                        motor.current = float(self.twos_complement_16bit((message.data[4] << 8) | message.data[5]))
                        motor.temp = float(message.data[6])

                        if motor.q==None:
                            motor.q = angle  
                        else:
                            if motor.q > np.pi*1.5 and angle < np.pi*0.5:
                                motor.count += 1
                            elif motor.q < np.pi*0.5 and angle > np.pi*1.5:
                                motor.count -= 1
                        motor.abs_q = (angle + 2*np.pi*motor.count)/motor.gear_ratio
                        motor.q = angle
                        msg.data = motor.get_states()
                        self.publisher_.publish(msg)
                    
            except KeyboardInterrupt:
                print("\nStopped by user")
                break
    
    def twos_complement_16bit(self, value):
        if value & (1 << 15):  # Check if the sign bit is set (16th bit)
            value = value - (1 << 16)  # Compute the two's complement
        return value


def main(args=None):
    rclpy.init(args=args)

    foc_data_publisher = focDataPublisher()

    rclpy.spin(foc_data_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    foc_data_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()