import rclpy
from rclpy.node import Node
# Importing the rclpy Node class to inherit several Noded characteristics and functions, such as create_subscription
from turtlesim.msg import Pose
# Importing Pose to specifiy that the subscribers created below will receive messages of the Pose type

class SimpleTurtleSimKinematics(Node):
    def __init__(self):
        super().__init__('simple_turtle_sim_kinematics')

        self.turtle1_pose_sub_ = self.create_subscription(
            Pose,                           # Type of message received by the subscriber
            '/turtle1/pose',                # Topics that the message is published to
            self.turtle1PoseCallback,     # Callback function that executes when this subscriber node receives a message on the topicc
            10                              # Queue size
        )
        self.turtle2_pose_sub_ = self.create_subscription(Pose, '/turtle2/pose', self.turtle2PoseCallback, 10)
        
        self.last_turtle1_pose = Pose()     # Creating two instances of the pose message to capture the last turtle pose
        self.last_turtle2_pose = Pose()

    def turtle1PoseCallback(self, msg):
        self.last_turtle1_pose_ = msg


    def turtle2PoseCallback(self, msg):
        self.last_turtle2_pose_ = msg

        tx = self.last_turtle2_pose.x - self.last_turtle1_pose.x
        ty = self.last_turtle2_pose.y - self.last_turtle1_pose.y
        self.get_logger().info("""\nTranslation Vector between Turtles:\n
                               Tx: %f\n
                               Ty: %f""" % (tx,ty))

def main():
    rclpy.init()
    simple_turtlesim_kinematics = SimpleTurtleSimKinematics()
    rclpy.spin(simple_turtlesim_kinematics)

    simple_turtlesim_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
