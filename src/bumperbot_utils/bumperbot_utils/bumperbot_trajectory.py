import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryDrawer(Node):
    def __init__(self):
        super().__init__("trajectory_drawer")

        self.trajectory_ = Path()
        self.trajectory_.header.frame_id = "odom"

        self.pose_ = PoseStamped()
        self.pose_.header.frame_id = "base_footprint"
        self.pose_.pose.position.x = 0.0
        self.pose_.pose.position.y = 0.0
        self.pose_.pose.position.z = 0.0
        self.pose_.pose.orientation.w = 1.0

        self.trajectory_.poses.append(self.pose_)

        self.trajectory_pub_ = self.create_publisher(Path,"bumperbot_controller/trajectory",10)
        self.odom_sub_ = self.create_subscription(Odometry,"bumperbot_controller/odom",self.odomCallback,10)

    def odomCallback(self, msg):
        self.pose_.pose.position = msg.pose.pose.position
        self.pose_.pose.orientation = msg.pose.pose.orientation

        self.trajectory_.poses.append(self.pose_)
        self.trajectory_pub_.publish(self.trajectory_)

def main():
    rclpy.init()
    bumperbot_trajectory = TrajectoryDrawer()
    rclpy.spin(bumperbot_trajectory)
    bumperbot_trajectory.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
