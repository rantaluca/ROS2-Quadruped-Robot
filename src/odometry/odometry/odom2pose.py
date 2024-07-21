#!/usr/bin/env python3

import numpy as np
import math 
import rospy
from geometry_msgs.msg import PoseStamped
from turtlebot3_msgs.msg import SensorState
from sensor_msgs.msg import Imu, MagneticField
from tf.transformations import quaternion_from_euler

def coordinates_to_message(x, y, O, t):
    msg = PoseStamped()
    msg.pose.position.x = x
    msg.pose.position.y = y
    [msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w] = quaternion_from_euler(0.0, 0.0, O)
    msg.header.stamp = t
    msg.header.frame_id = 'base_link'
    return msg

class Odom2PoseNode:
    def __init__(self):
        rospy.init_node('odom2pose')

        # Constants
        self.ENCODER_RESOLUTION = 4096
        self.WHEEL_RADIUS = 0.033
        self.WHEEL_SEPARATION = 0.160
        self.MAG_OFFSET = np.pi/2.0-0.07

        # Variables
        self.x_odom, self.y_odom, self.O_odom = 0, 0, 0
        self.x_gyro, self.y_gyro, self.O_gyro = 0, 0, 0
        self.x_magn, self.y_magn, self.O_magn = 0, 0, 0
        self.prev_left_encoder = 0
        self.prev_right_encoder = 0
        self.prev_gyro_t = 0
        self.v = 0

        # Publishers
        self.pub_gyro = rospy.Publisher('/pose_gyro', PoseStamped, queue_size=0)

        # Subscribers
        self.sub_gyro = rospy.Subscriber('/imu', Imu, self.callback_gyro)

    def callback_gyro(self, gyro):
        if self.v == 0:
            return

        # Compute the elapsed time
        t = gyro.header.stamp.to_sec()
        dt = t - self.prev_gyro_t
        if self.prev_gyro_t == 0:
            self.prev_gyro_t = t
            return
        self.prev_gyro_t = t

        # TODO: compute the angular velocity
        
        self.w = gyro.angular_velocity.z*dt

        # TODO: update O_gyro, x_gyro and y_gyro accordingly (using self.v)
        

        self.O_gyro = self.O_gyro + self.w 
        self.x_gyro = self.x_gyro  + self.v * dt  * math.cos(self.O_gyro)
        self.y_gyro = self.y_gyro + self.v * dt  * math.sin(self.O_gyro)
    

        msg = coordinates_to_message(self.x_gyro, self.y_gyro,self.O_gyro, gyro.header.stamp)
        self.pub_gyro.publish(msg)
        
if __name__ == '__main__':
    node = Odom2PoseNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass