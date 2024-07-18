import os
from os import pathsep
# pathsep is used for separating two paths

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
# IncludeLaunchDescription - allows for accessing other launch files in this launch file

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
# PythonLaunchDescriptionSouce - to include Python launch files

def generate_launch_description():

    bumperbot_description = get_package_share_directory("bumperbot_description")   # Full path to the bumperbot_description package
    bumperbot_description_prefix = get_package_prefix("bumperbot_description")      # Getting the path prefix of the bumperbot_description package

    model_path = os.path.join(bumperbot_description, "models")
    model_path += pathsep + os.path.join(bumperbot_description_prefix, "share")     

    env_variable = SetEnvironmentVariable("GAZEBO_MODEL_PATH", model_path)

    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(bumperbot_description, "urdf","bumperbot.urdf.xacro"),
        description = "Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(Command(["xacro ", LaunchConfiguration("model")]),value_type = str)

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description}]
    )

    start_gazebo_server = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch","gzserver.launch.py"
    )))
    start_gazebo_client = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory("gazebo_ros"), "launch","gzclient.launch.py"
    )))

    spawn_robot = Node(
        package = "gazebo_ros",
        executable = "spawn_entity.py",
        arguments = ["-entity", "bumperbot","-topic","robot_description"],
        output = "screen"
    )

    return LaunchDescription([
        env_variable,
        model_arg,
        robot_state_publisher_node,
        start_gazebo_server,
        start_gazebo_client,
        spawn_robot
    ])
