#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import TransformStamped, Vector3
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import numpy as np
import math

def degrees_to_radians(degrees):
    return degrees * (math.pi / 180.0)

class ImuTfBroadcaster(Node):
    def __init__(self):
        super().__init__('tf_broadcaster_imu')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('imu_frame_id', 'base_imu')

        self.orientation_data = None
        
        # state variables
        self.last_time = self.get_clock().now()
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.position = np.array([0.0, 0.0, 0.0])

        # Subscribe to orientation data topic
        self.subscription_orientation = self.create_subscription(
            Vector3,
            '/sensor/bwt901cl/Angle',
            self.orientation_callback,
            10)
        
        # Subscribe to linear accel data topic
        self.subscription_orientation = self.create_subscription(
            Imu,
            '/sensor/bwt901cl/Imu',
            self.linear_accel_callback,
            10)

        self.br = tf2_ros.TransformBroadcaster(self)


    def linear_accel_callback(self, msg):

        t = TransformStamped()

        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
        t.child_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value

        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        #  linear acceleration from imu
        linear_acceleration = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z - 9.81
        ])

        # velocity = velocity + acceleration * dt
        self.velocity += linear_acceleration * dt

        # position = position + velocity * dt + 0.5 * acceleration * dt^2
        self.position += self.velocity * dt + 0.5 * linear_acceleration * dt**2
        
        # position = position + velocity * dt
        #self.position += self.velocity * dt

        t.transform.translation.x = self.position[0]
        t.transform.translation.y = self.position[1]
        t.transform.translation.z = self.position[2]

        self.br.sendTransform(t)
        self.get_logger().info(f"Transform broadcasted from {t.header.frame_id} to {t.child_frame_id}")


    def orientation_callback(self, msg):
        self.orientation_data = msg
        self.broadcast_transform()

    def broadcast_transform(self):
        if self.orientation_data is None:
            return

        try:
            t = TransformStamped()

            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = self.get_parameter('base_frame_id').get_parameter_value().string_value
            t.child_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value

            # Convert Euler angles from degrees to radians
            roll = degrees_to_radians(self.orientation_data.x)
            pitch = degrees_to_radians(self.orientation_data.y)
            yaw = -degrees_to_radians(self.orientation_data.z)

            # Convert Euler angles to quaternion
            quaternion = quaternion_from_euler(roll, pitch, yaw)

            t.transform.rotation.x = quaternion[0]
            t.transform.rotation.y = quaternion[1]
            t.transform.rotation.z = quaternion[2]
            t.transform.rotation.w = quaternion[3]

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
