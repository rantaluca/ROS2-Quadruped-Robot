import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class InverseKinematicsPublisher(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Timer for periodic callbacks
        self.timer_period = 0.02  # 50 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Initialize joint state message
        self.joint_state = JointState()
        self.joint_state.name = [
            'Revolute 33', 'Revolute 32', 'Revolute 43',  # Back Left Leg
            'Revolute 38', 'Revolute 36', 'Revolute 46',  # Back Right Leg
            'Revolute 29', 'Revolute 30', 'Revolute 45',  # Front Left Leg
            'Revolute 37', 'Revolute 35', 'Revolute 47',  # Front Right Leg
        ]
        self.joint_state.position = [0.0] * 12

        # Leg parameters (adjust based on your robot)
        self.leg_params = {
            'l1': 0.087,  # Upper leg length
            'l2': 0.111,  # Lower leg length
        }

        # Gait parameters
        self.cycle_time = 1.0  # One second per gait cycle
        self.step_length = 0.07
        self.step_height = 0.04

        # Start time
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        # Compute elapsed time
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        leg_phase_offsets = [0.0, 0.25, 0.5, 0.75]  # Leg phase offsets for walking gait
        for i in range(4):  # For each leg
            leg_phase = ((current_time / self.cycle_time) + leg_phase_offsets[i]) % 1.0
            t_leg = leg_phase * self.cycle_time

            x, y = self.gait_generator(t_leg, self.cycle_time, self.step_length, self.step_height)
            theta1, theta2 = self.inverse_kinematics(x, y, self.leg_params)

            # Map to joint indices
            if i == 0:  # Back Left Leg
                self.joint_state.position[1] = -theta1  # Upper leg
                self.joint_state.position[0] = -theta2  # Lower leg
            elif i == 1:  # Front Left Leg
                self.joint_state.position[7] = -theta1
                self.joint_state.position[6] = -theta2
            elif i == 2:  # Back Right Leg
                self.joint_state.position[4] = theta1
                self.joint_state.position[3] = theta2
            elif i == 3:  # Front Right Leg
                self.joint_state.position[10] = theta1
                self.joint_state.position[9] = theta2

        # Set hip yaw/pitch joints
        self.joint_state.position[2] = 0.2  # Back Left Leg hip
        self.joint_state.position[5] = -0.2  # Back Right Leg hip
        self.joint_state.position[8] = 0.2  # Front Left Leg hip
        self.joint_state.position[11] = -0.2  # Front Right Leg hip

        # Update timestamp and publish
        self.joint_state.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state)

    def inverse_kinematics(self, x, y, leg_params):
        # Extract leg parameters
        l1 = leg_params['l1']
        l2 = leg_params['l2']

        # Compute the distance from the hip to the foot
        r = math.hypot(x, y)
        r = min(r, l1 + l2 - 0.01)  # Adjust for reach

        # Avoid numerical issues
        cos_theta2 = (r**2 - l1**2 - l2**2) / (2 * l1 * l2)
        cos_theta2 = max(min(cos_theta2, 1.0), -1.0)
        theta2 = math.acos(cos_theta2)

        # Compute theta1
        k1 = l1 + l2 * math.cos(theta2)
        k2 = l2 * math.sin(theta2)
        theta1 = math.atan2(y, x) - math.atan2(k2, k1)

        return theta1, math.pi - theta2

    def gait_generator(self, t, cycle_time, step_length, step_height):
        phase = (t % cycle_time) / cycle_time

        if phase < 0.5:
            # Swing phase (foot moves forward and upward)
            normalized_phase = phase / 0.5
            x = -step_length / 2 + step_length * normalized_phase
            y = step_height * math.sin(math.pi * normalized_phase)
        else:
            # Stance phase (foot moves backward and remains on the ground)
            normalized_phase = (phase - 0.5) / 0.5
            x = step_length / 2 - step_length * normalized_phase
            y = 0.0  # Foot is on the ground during stance

        return x, y

def main(args=None):
    rclpy.init(args=args)

    inverse_kinematics_publisher = InverseKinematicsPublisher()

    try:
        rclpy.spin(inverse_kinematics_publisher)
    except KeyboardInterrupt:
        pass

    inverse_kinematics_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
