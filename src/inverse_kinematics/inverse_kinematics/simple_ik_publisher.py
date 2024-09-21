import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math
import numpy as np

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
            'l1': 0.1,  # Upper leg length
            'l2': 0.1,  # Lower leg length
        }

        # Gait parameters
        self.cycle_time = 6.0  # One second per gait cycle
        self.step_length = 0.07
        self.step_height = 0.1

        # Start time
        self.start_time = self.get_clock().now()

    def timer_callback(self):
        # Compute elapsed time
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        # Generate gait for each leg
        for i in range(4):  # Four legs
            t = current_time + (i % 2) * self.cycle_time / 2  # Offset for trot gait
            x, y = self.gait_generator(t, self.cycle_time, self.step_length, self.step_height)

            # Inverse kinematics for leg
            theta1, theta2 = self.inverse_kinematics(x, y, self.leg_params)

            # Map to joint indices
            if i == 0:  # Back Left Leg
                self.joint_state.position[1] = theta1  # Upper leg
                self.joint_state.position[0] = theta2  # Lower leg
            elif i == 1:  # Back Right Leg
                self.joint_state.position[4] = theta1
                self.joint_state.position[3] = theta2
            elif i == 2:  # Front Left Leg
                self.joint_state.position[7] = theta1
                self.joint_state.position[6] = theta2
            elif i == 3:  # Front Right Leg
                self.joint_state.position[10] = theta1
                self.joint_state.position[9] = theta2

            # Elbow joint (hip yaw/pitch)
            self.joint_state.position[2 + i * 3] = 0.0  # Keep hip joint neutral

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

        return theta1,  math.pi - theta2

    def gait_generator(self, t, cycle_time, step_length, step_height):
        phase = (t % cycle_time) / cycle_time
        y = 0.0
        if phase < 0.5:
            # Swing phase
            x = step_length * (2 * phase - 1)
            y = step_height * np.sin(phase)

        else:
            # Stance phase
            x = -step_length * (2 * phase - 1)
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
