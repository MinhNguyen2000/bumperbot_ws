from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():

    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(get_package_share_directory("bumperbot_description"),"urdf","bumperbot.urdf.xacro"),
        description = "Absolute path to robot URDF file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type = str)
    # This line extract the URDF model from the path
    # 1. LaunchConfiguration() get the xacro URDF model from the launch argument "model"
    # 2. Command(["xacro ", ]) then convert this xacro URDF model to a conventional URDF model

    robot_state_publisher = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description}]
    )
    # This is equivalent to the terminal command >> ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro /home/minhnguyen/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro)"

    joint_state_publisher_gui = Node(
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui"
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments = ["-d", os.path.join(get_package_share_directory("bumperbot_description"),"rviz","display.rviz")]
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])