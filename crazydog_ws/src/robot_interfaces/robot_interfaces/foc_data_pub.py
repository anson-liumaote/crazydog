import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
import can
import math

GEAR_RATIO_M3508 = 15.76
GEAR_RATIO_M2006 = 36.0

class focDataPublisher(Node):

    def __init__(self):
        super().__init__('foc_data_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'foc_msg', 1)
        can_interface = 'can0'
        self.bus = can.interface.Bus(channel=can_interface, interface='socketcan')
        self.t0 = self.get_clock().now()
        self.angle_single = 0.0
        self.p0 = None
        self.receive_can_messages()

    def receive_can_messages(self):
        msg = Float32MultiArray()
        while True:
            try:
                message = self.bus.recv(timeout=1)
                t1 = self.get_clock().now()
                dt = (t1 - self.t0).nanoseconds / 1e9  # Time in seconds
                self.t0 = t1
                # for c620
                if message is not None and (message.arbitration_id==0x201 or message.arbitration_id==0x202):
                    id = float(message.arbitration_id)
                    angle = float(((message.data[0] << 8) | message.data[1])/8192*2*math.pi)/GEAR_RATIO_M3508
                    speed = float(self.twos_complement_16bit((message.data[2] << 8) | message.data[3]))/GEAR_RATIO_M3508 #19.2 old gear ratio #15.76 new gear ratio
                    current = float(self.twos_complement_16bit((message.data[4] << 8) | message.data[5]))
                    temperature = float(message.data[6])
                    print('id', id,'angle', angle, 'speed', speed, 'current', current, 'temp', temperature)
                    msg.data = [id, angle, speed, current, temperature]
                    self.publisher_.publish(msg)
                # for c610
                elif message is not None and (message.arbitration_id==0x203):
                    id = float(message.arbitration_id)
                    angle = float(((message.data[0] << 8) | message.data[1])/8192*2*math.pi)/GEAR_RATIO_M2006
                    speed = float(self.twos_complement_16bit((message.data[2] << 8) | message.data[3]))/GEAR_RATIO_M2006 # gear ratio 1:36
                    current = float(self.twos_complement_16bit((message.data[4] << 8) | message.data[5]))
                    temperature = 0.0
                    self.angle_single += speed * dt
                    # print('id', id,'angle', angle, 'speed', speed, 'current', current, 'temp', temperature)
                    if self.p0==None:
                        self.p0 = angle
                    print(angle, self.angle_single+self.p0)
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