import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

WHEEL_RADIUS = 0.07     # m
WHEEL_DISTANCE = 0.355

class focMotor():
    def __init__(self):
        self.angle = 0.0    # rad
        self.speed = 0.0    # rpm
        self.current = 0.0  
        self.temperature = 0.0  # degree C

class OdometryNode(Node):
    def __init__(self, wheels_separation, wheels_radius):
        super().__init__('odometry_node')
        
        self.wheel_radius = wheels_radius  # in meters
        self.wheel_separation = wheels_separation  # in meters

        self.jointstate = JointState()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.imu_subscriber = self.create_subscription(Imu, '/handsfree/imu', self.imu_callback, 10)
        self.jointstate_subscriber = self.create_subscription(JointState,'jointstate', self.jointstate_callback, 1)
        self.last_time = self.get_clock().now()
        # self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

    def jointstate_callback(self, msg):
        self.jointstate = msg

    def imu_callback(self, msg):
        try:
            v_left, v_right = self.jointstate.velocity[6], self.jointstate.velocity[7]
            self.publish_odometry(v_left, v_right, msg)
        except:
            self.get_logger().info('waiting for jointstate')

    def rpm_to_mps(self, rpm):
        """Convert RPM to meters per second."""
        return rpm * self.wheel_radius * (2 * math.pi / 60)

    def publish_odometry(self, v_left, v_right, imu_msg):
        # """Publish odometry based on left and right RPM."""
        # v_left = self.rpm_to_mps(rpm_left)
        # v_right = self.rpm_to_mps(rpm_right)

        v_avg = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_separation

        # Update the robot's position
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Time in seconds
        self.last_time = current_time
        self.x += v_avg * dt * math.cos(self.theta)
        self.y += v_avg * dt * math.sin(self.theta)
        self.theta += omega * dt

        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        odom_msg.pose.pose.orientation = imu_msg.orientation
        # odom_msg.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.theta)

        odom_msg.twist.twist = Twist()
        odom_msg.twist.twist.linear.x = v_avg
        odom_msg.twist.twist.angular.z = omega

        self.odom_pub.publish(odom_msg)

    # def quaternion_from_euler(self, roll, pitch, yaw):
    #     """Convert Euler angles to a quaternion."""
    #     qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    #     qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    #     qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    #     qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    #     return (qx, qy, qz, qw)

    


def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode(wheels_separation=WHEEL_DISTANCE, wheels_radius=WHEEL_RADIUS)  # example values
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
