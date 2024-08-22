import rclpy
from rclpy.node import Node
from math import *
from sensor_msgs.msg import JointState



class IkPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.i = 0
        self.j = 0
        self.A = 49
        self.E = 87
        self.F = 111

        self.crawl_switcher = {
            0: "fl",
            1: "br",
            2: "fr",
            3: "bl"
        }

        self.joints_dict = {"bl":["Revolute 43", "Revolute 32", "Revolute 33"],
                            "br":["Revolute 46", "Revolute 36", "Revolute 38"],
                            "fl":["Revolute 45", "Revolute 30", "Revolute 29"],
                            "fr":["Revolute 47", "Revolute 35", "Revolute 37"]}
        
        # a joint state publisher that compute the ik and publish the joint states
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 10)

        self.gait_1_points = [
                (0, 0, 130),
                (-30, 0,90 ),
                (-40, 0,60),
                (0, 0, 0),
                (40, 0,60),
                (60, 0,90),
                (60, 0,100),
                (60, 0,110),
                (60, 0,130)
            ]
        # a timer to publish the joint states
        self.timer = self.create_timer(0.8, self.timer_callback)

    # def timer_callback(self):
    #     # send the commands fl first than br than fr than bl
    #     if self.i == 3:
    #         self.i = 0
    #     if self.j == len(self.gait_1_points):
    #         self.j = 0
    #         self.i += 1
    #     leg = self.crawl_switcher.get(self.i)
    #     omega, theta, phi = self.inverse_kinematics(self.gait_1_points[self.j][0], self.gait_1_points[self.j][1], self.gait_1_points[self.j][2])
    #     self.publish_joint_states(leg, omega, theta, phi)
    #     self.j += 1

    def timer_callback(self):
        # Send the commands: fl first, then br, then fr, then bl
        if self.i == 3:
            self.i = 0
        if self.j == len(self.gait_1_points):
            self.j = 0
            self.i += 1
        
        leg = self.crawl_switcher.get(self.i)
        # Calculate the inverse kinematics for the current point
        omega, theta, phi = self.inverse_kinematics(
            self.gait_1_points[self.j][0], 
            self.gait_1_points[self.j][1], 
            self.gait_1_points[self.j][2]
        )

        # Only publish if the IK solution is valid (i.e., not None)
        if omega is not None and theta is not None and phi is not None:
            self.publish_joint_states(leg, omega, theta, phi)
        else:
            self.get_logger().warn(f"Skipping publication for {leg} due to unreachable point.")

        self.j += 1
        


    #publish the joint states for the leg
    
    def publish_joint_states(self, leg, omega, theta, phi):
        # Create a JointState message
        joint_state = JointState()

        # Fill the JointState message
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.joints_dict[leg]  # Joint names for the leg
        joint_state.position = [omega, theta, phi]  # Corresponding joint angles

        # Publish the joint state
        self.publisher_.publish(joint_state)
        self.get_logger().info(f"Published joint states for {leg}: omega={omega}, theta={theta}, phi={phi}")


    # def inverse_kinematics(self,X,Y,Z):

    #     # computing elbow angle
    #     C = sqrt(Y**2 + Z**2)
    #     D = sqrt(C**2 - self.A**2)
    #     alpha = atan(Z/Y)
    #     beta = atan(D/self.A)
    #     omega = alpha + beta # elbow angle

    #     # computing knee angle
    #     G = sqrt(D**2 + X**2)   
    #     phi = acos((G**2 - self.E**2 - self.F**2)/(-2*self.E*self.F)) # knee angle
    #     # computing shoulder angle

    #     alpha2 = atan(X/D)
    #     beta2 = asin((sin(phi)*self.F)/G)
    #     theta = alpha2 + beta2

    #     return omega, theta, phi

    def inverse_kinematics(self, X, Y, Z):
        try:
            # computing elbow angle
            C = sqrt(Y**2 + Z**2)
        except ValueError:
            self.get_logger().error(f"Point ({X}, {Y}, {Z}) is not reachable: sqrt of negative value for C.")
            return None, None, None

        try:
            D = sqrt(C**2 - self.A**2)
        except ValueError:
            self.get_logger().error(f"Point ({X}, {Y}, {Z}) is not reachable: sqrt of negative value for D.")
            return None, None, None

        try:
            alpha = atan(Z/ Y)  # Use atan2 to avoid division by zero issues
            beta = atan(D /self.A)
            omega = alpha + beta  # elbow angle
        except ValueError:
            self.get_logger().error(f"Point ({X}, {Y}, {Z}) caused an error in atan calculation.")
            return None, None, None

        try:
            # computing knee angle
            G = sqrt(D**2 + X**2)   
            phi = acos((G**2 - self.E**2 - self.F**2) / (-2 * self.E * self.F))  # knee angle
        except ValueError:
            self.get_logger().error(f"Point ({X}, {Y}, {Z}) is not reachable: acos domain error for knee angle.")
            return None, None, None

        try:
            # computing shoulder angle
            alpha2 = atan(X/D)
            beta2 = asin((sin(phi) * self.F) / G)
            theta = alpha2 + beta2
        except ValueError:
            self.get_logger().error(f"Point ({X}, {Y}, {Z}) caused an error in asin calculation.")
            return None, None, None

        return omega, theta, phi





def main(args=None):
    rclpy.init(args=args)

    ik_publisher = IkPublisher()

    rclpy.spin(ik_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ik_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()