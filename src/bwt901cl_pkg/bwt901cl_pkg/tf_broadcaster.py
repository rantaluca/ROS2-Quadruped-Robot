#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, Vector3

class ImuTfBroadcaster(Node):
    def __init__(self):
        print("hello")
        super().__init__('tf_broadcaster_imu')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'base_imu')

        self.orientation_data = None

        # Subscribe to orientation data topic
        self.subscription_orientation = self.create_subscription(
            Vector3,
            '/imu/angle',
            self.handle_orientation_data,
            10)

        self.br = tf2_ros.TransformBroadcaster(self)

    def handle_orientation_data(self, msg):
        self.orientation_data = msg
        self.broadcast_transform()

    def broadcast_transform(self):
        try:
            t = TransformStamped()
            print("hello")

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
            t.child_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value

            # Use the orientation from the orientation data
            t.transform.rotation.x = self.orientation_data.x
            t.transform.rotation.y = self.orientation_data.y
            t.transform.rotation.z = self.orientation_data.z
            t.transform.rotation.w = self.orientation_data.w

            self.br.sendTransform(t)
            self.get_logger().info(f"Transform broadcasted from {t.header.frame_id} to {t.child_frame_id}")
        except Exception as e:
            self.get_logger().error(f"Error broadcasting transform: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuTfBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
