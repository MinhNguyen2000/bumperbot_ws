import os   # To manipulate file paths

# From the launch library (for managing the launch process in ROS2)
from launch import LaunchDescription                                # Defines the structure of the launch script and list the nodes to be run
from launch.actions import DeclareLaunchArgument                    # To parse arguments that user can pass into the launch file when executing the launch file
from launch.substitutions import Command, LaunchConfiguration       # Command - allows execution of a command-line instruction during the launch process (used later for converting xacro to URDF)
                                                                    # LaunchConfiguration - to handle launch arguments (used for obtaining the value of a launch argument that is defined when executing the launch file)

# From the launch_ros library (for configuring ROS2 specific functionalities)
from launch_ros.actions import Node                                 # To create ROS2 nodes
from launch_ros.parameter_descriptions import ParameterValue        # To create a parameter value
from ament_index_python.packages import get_package_share_directory # Retrieve the absolute path to the share directory of a specified package


def generate_launch_description():

    bumperbot_description_dir = get_package_share_directory("bumperbot_description")

    model_arg = DeclareLaunchArgument(
        name = "model",
        description = "Absolute path to robot URDF file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]), value_type = str)
    # This line creates the robot_description argument for the robot_state_publisher node
    # 1. LaunchConfiguration() get the xacro URDF model from the launch argument "model"
    # 2. Command(["xacro ", ]) then convert this xacro URDF model to a conventional URDF model
    # The variable robot_description is essentially "$( xacro /home/minhnguyen/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro)"


    robot_state_publisher = Node(
    # Opening the robot state publisher, which obtain the URDF model of the robot (its description)
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description}]
    )
    # This is equivalent to the terminal command >> ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$( xacro /home/minhnguyen/bumperbot_ws/src/bumperbot_description/urdf/bumperbot.urdf.xacro)"

    joint_state_publisher_gui = Node(
    # Open the GUI for controlling the joint states
        package = "joint_state_publisher_gui",
        executable = "joint_state_publisher_gui"
    )

    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "screen",
        arguments = ["-d", os.path.join(bumperbot_description_dir,"rviz","display.rviz")]
    )
    # The argument of the rviz node is to open the configuration file that we saved earlier

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz_node
    ])