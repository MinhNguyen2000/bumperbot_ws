import os
from os import pathsep
from pathlib import Path
# pathsep is used for separating two paths

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
# SetEnvironmentVariable - to set an environmental variable in Linux
# IncludeLaunchDescription - allows for accessing other launch files in this launch file

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
# PythonLaunchDescriptionSouce - to include Python launch files

def generate_launch_description():

    bumperbot_description_dir = get_package_share_directory("bumperbot_description")   # Full directory to the bumperbot_description package  
    ros_distro = os.environ["ROS_DISTRO"]
    is_ignition = "True" if ros_distro == "humble" else "False"

    model_arg = DeclareLaunchArgument(
        name = "model",
        default_value = os.path.join(bumperbot_description_dir, "urdf","bumperbot.urdf.xacro"),
        description = "Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(
        Command([
            "xacro ", LaunchConfiguration("model"),
            " is_ignition:=", is_ignition
            ]),
        value_type = str
    )

    robot_state_publisher_node = Node(
        package = "robot_state_publisher",
        executable = "robot_state_publisher",
        parameters = [{"robot_description": robot_description}]
    )

    # Set an environmental variable in Linux from a launch file
    gazebo_resource_path = SetEnvironmentVariable(
        name = "GZ_SIM_RESOURCE_PATH", 
        value = str(Path(bumperbot_description_dir).parent.resolve())
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory("ros_gz_sim"), "launch"),"/gz_sim.launch.py"]),
        launch_arguments = [("gz_args", ["-v 4", " -r"])]
    )

    gz_spawn_entity = Node(
        package = "ros_gz_sim",
        executable = "create",
        arguments = [
            "-topic", "robot_description",
            "-name", "bumperbot",
        ],
        output = "screen"
    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        gazebo_resource_path,
        gazebo,
        gz_spawn_entity
    ])
