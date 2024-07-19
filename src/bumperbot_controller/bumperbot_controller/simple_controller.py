#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
import numpy as np

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius",0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info("Using wheel_radius %f" % self.wheel_radius_)
        self.get_logger().info("Using wheel_separation %f" % self.wheel_separation_)

        self.wheel_cmd_pub_ = self.create_publisher(
            Float64MultiArray,                          # Type of the message published in the simple_velocity_controller/commands topic
            "simple_velocity_controller/commands",      # The topic over which the simple_velocity_controller sends the velocity commands to the wheels on the gazebo simulation
            10
        )

        self.vel_sub_ = self.create_subscription(
            TwistStamped,                               # Type of the messaged published in the bumperbot_controller/cmd_vel topic
            "bumperbot_controller/cmd_vel",
            self.velCallback,
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

        robot_speed = np.array([[msg.twist.linear.x],
                               [msg.twist.angular.z]])
        # robot_speed is the array containing the extracted values from the received message from the bumperbot_controller/cmd_vel
        # The velocity component aligned with the x axis of the robot and the angular velocity about the z axis of the robot are extracted

        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
        # Calculated the required wheel speed to obtain the specified robot velocity
        # Next, need to send these velocities as a message on the topic simple_velocity_controller/commands

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