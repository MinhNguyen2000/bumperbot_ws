import rclpy
from rclpy.node import Node                             # Importing this module to create a function that inherits the functionalities of a node
from std_msgs.msg import String                         # The type of the message subscribed by this node

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__("simple_subscriber")

        self.sub_ = self.create_subscription(
            # Using the create_subscription function (inherited from the Node class) to create a subscriber object
            String,                                     # Type of the message
            "chatter",                                  # Name of the topic
            self.msgCallback,                           # The callback function that is executed everytime the node receives a message
            10                                          # Size of the queue
        )

    def msgCallback(self, msg):
        self.get_logger().info("I heard: %s" % msg.data)

def main():
    rclpy.init()
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)

    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()