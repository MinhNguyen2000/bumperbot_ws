import rclpy
from rclpy.node import Node
# Importing the rclpy Node class to inherit several Noded characteristics and functions, such as create_subscription
from turtlesim.msg import Pose
# Importing Pose to specifiy that the subscribers created below will receive messages of the Pose type
import math

class SimpleTurtleSimKinematics(Node):
    def __init__(self):
        super().__init__('simple_turtle_sim_kinematics')

        self.turtle1_pose_sub_ = self.create_subscription(
            Pose,                           # Type of message received by the subscriber
            '/turtle1/pose',                # Topics that the message is published to
            self.turtle1PoseCallback,     # Callback function that executes when this subscriber node receives a message on the topicc
            10                              # Queue size
        )
        self.turtle2_pose_sub_ = self.create_subscription(
            Pose, 
            '/turtle2/pose', 
            self.turtle2PoseCallback, 
            10
        )
        
        self.last_turtle1_pose_ = Pose()     # Creating two instances of the pose message to capture the last turtle pose
        self.last_turtle2_pose_ = Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose_ = msg

    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose_ = msg

        tx = self.last_turtle2_pose_._x - self.last_turtle1_pose_.x
        ty = self.last_turtle2_pose_.y - self.last_turtle1_pose_.y

        theta_rad = self.last_turtle2_pose_.theta - self.last_turtle1_pose_.theta
        theta_deg = 180 * theta_rad / 3.14

        self.get_logger().info("""\n
                               Translation Vector between Turtles:\n
                               Tx: %f\n
                               Ty: %f\n 
                               Rotation Matrix between Turtles \n
                               theta(rad): %f \n
                               theta(deg): %f \n
                               |R11     R12| : |%7.3f     %7.3f|\n
                               |R21     R22| : |%7.3f     %7.3f|""" % (tx,ty,theta_rad,theta_deg,
                                                                 math.cos(theta_rad),-math.sin(theta_rad),
                                                                 math.sin(theta_rad),math.cos(theta_rad)))

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtleSimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)

    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
