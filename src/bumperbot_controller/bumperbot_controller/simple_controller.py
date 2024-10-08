#!/usr/bin/env python3
# The directory of the Python interpreter (as the build type) such that the ament_cmake interpreter cannot detect it automatically

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped             
# For subscribing to TwistStamped messages from bumperbot_controller/cmd_vel
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry 
import math
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster

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

        # Support variables for robot differential inverse kinematics
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0
        self.prev_time_ = self.get_clock().now()

        # Support variables for robot position and orientation
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0

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

        # Subscriber object for the topic /joint_states
        self.joint_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)

        # Publisher object to publish the odometry messages
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom",10)

        self.speed_conversion_ = np.array([
            [self.wheel_radius_ / 2, self.wheel_radius_ / 2],
            [self.wheel_radius_ / self.wheel_separation_, -self.wheel_radius_ / self.wheel_separation_]
        ])
        
        # Initialization of the Odometry message. Updated in jointCallback
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        self.transform_stamped_.header .frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint"

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

    def jointCallback(self, msg):
        # Robot velocity calculation from encoder sensor measurement
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_

        self.right_wheel_prev_pos_ = msg.position[0]
        self.left_wheel_prev_pos_ = msg.position[1] 
        self.prev_time_ = Time.from_msg(msg.header.stamp)

        phi_right = dp_right/(dt.nanoseconds / S_TO_NS)
        phi_left = dp_left/(dt.nanoseconds / S_TO_NS)

        # Linear and angular velocity of the robot
        linear = self.wheel_radius_/2 * (phi_right + phi_left)
        angular = self.wheel_radius_ / (self.wheel_separation_) * (phi_right - phi_left)

        # Position and orientation of the robot
        d_s = self.wheel_radius_ / 2 * (dp_right + dp_left)
        d_theta = self.wheel_radius_/self.wheel_separation_*(dp_right - dp_left)

        self.theta_ += d_theta
        self.x_ += d_s * math.cos(self.theta_)
        self.y_ += d_s * math.sin(self.theta_)

        # Updating the odometry message such as the current position/orientation(pose), and velocity (twist) of [base_footprint] wrt [odom]
        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        # Updating the transformation between [base_footprint] and [odom]
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        self.odom_pub_.publish(self.odom_msg_)
        self.br_.sendTransform(self.transform_stamped_)

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()