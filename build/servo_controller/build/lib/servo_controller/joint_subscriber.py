import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from adafruit_servokit import ServoKit

class JointStateSubscriber(Node):
    def __init__(self):
        super().__init__('joint_state_subscriber')
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.pca = ServoKit(channels=16)
        for i in range(0, 12):
            self.pca.servo[i].actuation_range = 270

    def listener_callback(self, msg):
        motor_mapping = {
            'lower_leg_bl': 0,
            'upper_leg_bl': 1,
            'elbow_bl': 2,
            'lower_leg_br': 3,
            'upper_leg_br': 4,
            'elbow_br': 5,
            'lower_leg_fl': 6,
            'upper_leg_fl': 7,
            'elbow_fl': 8,
            'lower_leg_fr': 9,
            'upper_leg_fr': 10,
            'elbow_fr': 11,
        }
        for name, position in zip(msg.name, msg.position):
            if name in motor_mapping:
                motor_index = motor_mapping[name]
                self.pca.servo[motor_index].angle = self.convert_to_servo_angle(position)

    def convert_to_servo_angle(self, joint_position):
        # Implement the conversion from joint position to servo angle
        return joint_position * (180.0 / 3.14159)  # Example conversion, adjust as needed

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
