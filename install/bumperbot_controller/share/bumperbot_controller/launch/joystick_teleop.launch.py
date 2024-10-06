import os

from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# THE CONTROL PIPELINE:
# 1. User use the joysticks on a PS4 controller, that message is read and published on a ROS2 topic by the node "joystick" as a "Joy" message
# 2. The published message is read by the "joy_teleop" node, converted to a TwistStamped message, and then published to the "bumperbot_controller/cmd_vel" topic
# 3. The "simple_controller" node subscribes to the "bumperbot_controller/cmd_vel" topic, converts the robot velocity to the required wheel speed
# 4. The wheel speed is published to the topic "simple_velocity_controller/commands", which controls and actuate the wheel

def generate_launch_description():

    joy_node = Node(
    # Joystick driver node - This node reads the joystick command and publishes it within a ROS2 topic
        package = "joy",
        executable = "joy_node",
        name = "joystick",
        parameters = [os.path.join(get_package_share_directory("bumperbot_controller"),"config","joy_config.yaml")]
    )


    joy_teleop = Node(
    # This node converts the joystick command of a PS4 controller (published by the node above) to a TwistStamped message
    # This message type is expected by the bumperbot_controller/cmd_vel node
        package = "joy_teleop",
        executable = "joy_teleop",
        parameters = [os.path.join(get_package_share_directory("bumperbot_controller"),"config","joy_teleop.yaml")]
    )

    return LaunchDescription([
        joy_node,
        joy_teleop
    ])