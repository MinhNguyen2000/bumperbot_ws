import rclpy
from rclpy.node import Node                             # Importing this module to create a function that inherits the functionality of a node
from std_msgs.msg import String                         # The type of the message published by this node

class SimplePublisher(Node):
    def __init__(self):
        super().__init__("simple_publisher")

        self.pub_ = self.create_publisher(
        # Using the create_publisher function (inherited from the Node class) to create a publisher object
            String,                                     # Type of the message
            "chatter",                                  # Name of the topic
            10                                          # Size of the queue
        )

        self.counter_ = 0
        self.frequency_ = 1.0                           # The frequency of publishing the message

        self.get_logger().info("Publishing at %d Hz" % self.frequency_)

        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)
        # Using the create_timer function (inherited from the Node class) to create a timer object that execute a function when the timer runs out
        # The frequency of the timerr is defined as self.frequency_ 

    def timerCallback(self):
        msg = String()
        msg.data = "Hello ROS2 - counter: %d" % self.counter_
        # The msg variable is of type String(), of which we can modify by assigning a value to the "data" attribute

        self.pub_.publish(msg)                          # Use the publisher object (created within the constructor) to publish the message
        self.counter_ += 1

def main():
    rclpy.init()                                        # Initialize the ROS2 interface
    simple_publisher = SimplePublisher()                # Instanttiate a new object of the SimplePublisher() class
    rclpy.spin(simple_publisher)                        # Keep spinning the node (or keep it running)

    simple_publisher.destroy_node()                     # Properly destroy the node when we terminate the execution of the node
    rclpy.shutdown()                                    # Properly shutdown the ROS2 interface

if __name__ == "main":
    main()