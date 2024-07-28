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
            'Revolute 33': 0,#lower_leg_bl
            'Revolute 32': 1,#upper_leg_bl
            'Revolute 43': 2,#elbow_bl
            'Revolute 38': 3,#lower_leg_br
            'Revolute 36': 4,#upper_leg_br
            'Revolute 46': 5,#elbow_br
            'Revolute 29': 6,#lower_leg_fl
            'Revolute 30': 7,#upper_leg_fl
            'Revolute 45': 8,#elbow_fl
            'Revolute 37': 9,#lower_leg_fr
            'Revolute 35': 10, #upper_leg_fr
            'Revolute 47': 11,#elbow_fr
        }
        for name, position in zip(msg.name, msg.position):
            if name in motor_mapping:
                motor_index = motor_mapping[name]
                self.pca.servo[motor_index].angle = self.convert_to_servo_angle(position)

    def convert_to_servo_angle(self, joint_position):
        #convert to 0-270deg
        angle = joint_position * 180 / 3.14159 - 135
        if angle > 270:
            angle = 270
        elif angle < 0:
            angle = 0
        return angle

def main(args=None):
    rclpy.init(args=args)
    joint_state_subscriber = JointStateSubscriber()
    rclpy.spin(joint_state_subscriber)
    joint_state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
