import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState
from scipy.spatial.transform import Rotation as R
import numpy as np

# WHEEL_RADIUS = 0.07     # m
# WHEEL_DISTANCE = 0.355

class focMotor():
    def __init__(self):
        self.angle = 0.0    # rad
        self.speed = 0.0    # rpm
        self.current = 0.0  
        self.temperature = 0.0  # degree C

class OdometryNode(Node):
    def __init__(self):
        super().__init__('odometry_node')

        # Declare the parameters with default values (in case they are not set from YAML)
        self.declare_parameter('wheel_radius')
        self.declare_parameter('wheel_distance')

        # Get the parameters from the YAML file or default values
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_distance = self.get_parameter('wheel_distance').value

        self.jointstate = JointState()

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # Orientation

        self.odom_pub = self.create_publisher(Odometry, 'odom', 1)
        self.imu_subscriber = self.create_subscription(Imu, '/handsfree/imu', self.imu_callback, 5)
        self.jointstate_subscriber = self.create_subscription(JointState,'jointstate', self.jointstate_callback, 5)
        self.last_time = self.get_clock().now()
        # self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz
        self.last_pos_left = None
        self.last_pos_right = None

    def jointstate_callback(self, msg):
        self.jointstate = msg

    def imu_callback(self, msg):
        try:
            pos_left, pos_right = self.jointstate.position[6], self.jointstate.position[7]
            v_left, v_right = self.jointstate.velocity[6] * self.wheel_radius, self.jointstate.velocity[7] * self.wheel_radius
            self.publish_odometry(pos_left, pos_right, v_left, v_right, msg)
        except Exception as e:
            self.get_logger().warning(f'{e}')

    def rpm_to_mps(self, rpm):
        """Convert RPM to meters per second."""
        return rpm * self.wheel_radius * (2 * math.pi / 60)

    def quaternion_multiply(self, q1, q2):
        # 四元數乘法
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    def rotate_quaternion_90_z(self, q):
        r, p, y = self.euler_from_quaternion(q[0], q[1], q[2], q[3])
        rn = p
        pn = -r 
        # yn = y + np.pi/2
        yn = self.theta
        qn = self.get_quaternion_from_euler(rn, pn, yn)
        return qn
    
    def euler_from_quaternion(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians

    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return [qx, qy, qz, qw]

    def publish_odometry(self, pos_left, pos_right, v_left, v_right, imu_msg: Imu):
        # """Publish odometry based on left and right RPM."""
        # v_left = self.rpm_to_mps(rpm_left)
        # v_right = self.rpm_to_mps(rpm_right)

        # Calculate the distance each wheel has traveled
        if self.last_pos_left == None or self.last_pos_right == None:
            self.last_pos_left = pos_left
            self.last_pos_right = pos_right
            return None
        else:
            dL = (pos_left - self.last_pos_left) * self.wheel_radius
            dR = (pos_right - self.last_pos_right) * self.wheel_radius
            self.last_pos_left = pos_left
            self.last_pos_right = pos_right

        v_avg = (v_left + v_right) / 2.0
        omega = (v_right - v_left) / self.wheel_distance

        # Calculate the forward movement and angular change
        d = (dL + dR) / 2.0
        dtheta = (dR - dL) / self.wheel_distance

        # Update position
        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)
        self.theta += dtheta  # Update orientation

        # Normalize theta to stay within -pi to pi range
        self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi

        # Create the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"

        odom_msg.pose.pose = Pose()
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        imu_orientation = [imu_msg.orientation.x, imu_msg.orientation.y, imu_msg.orientation.z, imu_msg.orientation.w]

        [odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, \
            odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w] = \
                self.rotate_quaternion_90_z(imu_orientation)
        
        odom_msg.twist.twist = Twist()
        odom_msg.twist.twist.linear.x = v_avg
        odom_msg.twist.twist.angular.z = omega
        odom_msg.twist.twist.angular.y = -imu_msg.angular_velocity.x     # !!! check according to imu pos

        self.odom_pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    odometry_node = OdometryNode()  # example values
    rclpy.spin(odometry_node)
    odometry_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
