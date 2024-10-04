import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import can
import math

class focDataPublisher(Node):

    def __init__(self):
        super().__init__('foc_data_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'foc_msg', 1)
        can_interface = 'can0'
        self.bus = can.interface.Bus(channel=can_interface, interface='socketcan')
        self.receive_can_messages()

    def receive_can_messages(self):
        msg = Float32MultiArray()
        while True:
            try:
                message = self.bus.recv(timeout=1)
                if message is not None and (message.arbitration_id==0x201 or message.arbitration_id==0x202):
                    id = float(message.arbitration_id)
                    angle = float(((message.data[0] << 8) | message.data[1])/8192*2*math.pi)
                    speed = float(self.twos_complement_16bit((message.data[2] << 8) | message.data[3]))/15.76 #19.2 old gear ratio #15.76 new gear ratio
                    current = float(self.twos_complement_16bit((message.data[4] << 8) | message.data[5]))
                    temperature = float(message.data[6])
                    print('id', id,'angle', angle, 'speed', speed, 'current', current, 'temp', temperature)
                    msg.data = [id, angle, speed, current, temperature]
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