#!/usr/bin/env python3
# The directory of the Python interpreter (as the build type) such that the ament_cmake interpreter cannot detect it automatically

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped              # For subscribing to TwistStamped messages from bumperbot_controller/cmd_vel
import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        # Declare parameters with default values
        self.declare_parameter("wheel_radius",0.033)
        self.declare_parameter("wheel_separation", 0.17)

        # Read the runtime value of parameters (if not then store the default values)
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        # Print the parameter value in terminal
        self.get_logger().info("Using wheel_radius %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel_separation %f" % self.wheel_separation_)

        # Publisher object to publish velocity commands to the wheels (message to simple_velocity_controller/commands)
        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray,                          # Type of the message published in the simple_velocity_controller/commands topic
            "simple_velocity_controller/commands",      # The topic over which the simple_velocity_controller sends the velocity commands to the wheels on the gazebo simulation
            10                                          # Buffer size
        )

        # Subscriber object that retrieve the robot velocity command (message from bumperbot_controller/cmd_vel)
        self.vel_sub_ = self.create_subscription(
            TwistStamped,                               # Type of the messaged published in the bumperbot_controller/cmd_vel topic
            "bumperbot_controller/cmd_vel",
            self.velCallback,                           # Callback function whenever receiving a message
            10
        )

        self.speed_conversion_ = np.array([
            [self.wheel_radius_ / 2, self.wheel_radius_ / 2],
            [self.wheel_radius_ / self.wheel_separation_, -self.wheel_radius_ / self.wheel_separation_]
        ])

        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

    def velCallback(self, msg):
        # Whenever this function is called (when a message is received on the bumperbot_controllerr/cmd_vel topic):
        # 1. Extract the linear and angular velocity components of the message
        # 2. Calculate the required wheel speed using the speed_conversion_ matrix
        # 3. Send the wheel speed command to the topic simple_velocity_controller/commands

        # 1. Robot velocity command extraction
        robot_speed = np.array([[msg.twist.linear.x],
                               [msg.twist.angular.z]])
            # robot_speed is the array containing the extracted values from the received message from the bumperbot_controller/cmd_vel
            # The velocity component aligned with the x axis of the robot and the angular velocity about the z axis of the robot are extracted

        # 2. Calculation of the required wheel speeds
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
            # Calculated the required wheel speed to obtain the specified robot velocity
            # Next, need to send these velocities as a message on the topic simple_velocity_controller/commands

        # 3. Publish the wheel velocity commands to the simple_velocity_controller/commands
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()