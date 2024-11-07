import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
import onnxruntime as ort
import numpy as np
from scipy.spatial.transform import Rotation as R
# from spatialmath import UnitQuaternion

class ModelInferenceNode(Node):
    def __init__(self):
        super().__init__('model_inference_node')
        
        # Initialize ONNX model
        self.model_path = '/home/crazydog/crazydog/crazydog_ws/src/rl_control/rl_control/model/2024-11-07_11-19-35/exported/policy.onnx'
        self.ort_session = ort.InferenceSession(self.model_path)
        
        # ROS 2 Subscribers
        self.odometry_sub = self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.joint_states_sub = self.create_subscription(JointState, '/jointstate', self.joint_states_callback, 10)
        
        # Initialize observation variables
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        self.projected_gravity = np.zeros(3)  # Update this based on your sensor data or estimation
        self.velocity_commands = np.zeros(3)  # Define as needed
        self.joint_pos = np.zeros(2)
        self.joint_vel = np.zeros(4)
        self.actions = np.zeros(4)  # Initial actions

    def get_projected_gravity(self, odom_msg:Odometry):
        # Extract the quaternion (x, y, z, w) from odometry orientation
        qx = odom_msg.pose.pose.orientation.x
        qy = odom_msg.pose.pose.orientation.y
        qz = odom_msg.pose.pose.orientation.z
        qw = odom_msg.pose.pose.orientation.w
        
        # Define gravity vector in the world frame
        gravity_world = np.array([0, 0, -1])  # Standard gravity

        # Convert quaternion to rotation matrix
        rotation = R.from_quat([qx, qy, qz, qw])
        rotation_matrix = rotation.as_matrix()

        # Rotate the gravity vector into the robot's frame
        projected_gravity = rotation_matrix.T @ gravity_world  # Transpose to rotate into base frame

        return projected_gravity

    def odometry_callback(self, msg:Odometry):
        # Extract linear and angular velocity from odometry message
        self.base_lin_vel[0] = msg.twist.twist.linear.x
        self.base_lin_vel[1] = msg.twist.twist.linear.y
        self.base_lin_vel[2] = msg.twist.twist.linear.z
        self.base_ang_vel[0] = msg.twist.twist.angular.x
        self.base_ang_vel[1] = msg.twist.twist.angular.y
        self.base_ang_vel[2] = msg.twist.twist.angular.z
        # Update other inputs as needed, e.g., projected gravity
        self.projected_gravity = self.get_projected_gravity(msg)

    def joint_states_callback(self, msg:JointState):
        # Assuming the first two are positional joints, and next four are velocity joints
        self.joint_pos = np.array([-msg.position[1], -msg.position[4]])-np.array([-1.27, -1.27])
        self.joint_vel = np.array([-msg.velocity[1], -msg.velocity[4], -msg.velocity[6],  msg.velocity[7]])

        # Perform inference once we have all data
        self.infer_model()

    def infer_model(self):
        # Prepare the input as a single (22,) array
        input_data = np.concatenate([
            self.base_lin_vel,
            self.base_ang_vel,
            self.projected_gravity,
            self.velocity_commands,
            self.joint_pos,
            self.joint_vel,
            self.actions
        ]).astype(np.float32).reshape(1, -1)  # Model expects (1, 22)

        # Run inference
        input_name = self.ort_session.get_inputs()[0].name
        outputs = self.ort_session.run(None, {input_name: input_data})
        
        # Process output (e.g., actions) as needed for control
        self.actions = outputs[0].flatten()  # Assuming model output is (1, 4) for actions

        # Optionally publish or use actions to control the robot
        print(input_data)
        print(self.actions)

def main(args=None):
    rclpy.init(args=args)
    node = ModelInferenceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
