from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
# To declare launch arguments (parameter) with some default value or can have a runtime value
# To complete a group action

from launch.substitutions import LaunchConfiguration    
# To read the runtime value of arguments

from launch.conditions import IfCondition, UnlessCondition
# The condition deciding whether to run the Python or C++ simple_controller executable

from launch_ros.actions import Node
# To inherit the functionality of the ROS2 Node class

def generate_launch_description():

    # Runtime Argument Definition
    use_python_arg = DeclareLaunchArgument(
        name = "use_python",
        default_value="True",           # By default, run the Python scripts (not C++)
        description="Whether to use Python implementation"
    )

    wheel_radius_arg = DeclareLaunchArgument(
        name = "wheel_radius",
        default_value = "0.033",
        description="Radius of the wheels"
    )

    wheel_separation_arg = DeclareLaunchArgument(
        name = "wheel_separation",
        default_value = "0.17",
        description="The baseline distance between the parallel wheels"
    )

    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value = "True"
    )

    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")
    
    # Node definition
    joint_state_broadcaster_spawner = Node(
            package = "controller_manager",
            executable = "spawner",
            arguments = [
                "joint_state_broadcaster",
                "--controller-manager",
                "/controller_manager"
            ]
    )

    # Node for spawing a bumperbot_controller that use the DiffDriveController
    wheel_controller_spawner = Node(
        package = "controller_manager",
        executable = "spawner",
        arguments = [
            "bumperbot_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition = UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition = IfCondition(use_simple_controller),
        actions = [

            # Spawn a simple_velocity_controller that receives wheel velocity messages and actuates the wheel
            # Subscribes to messages on the simple_velocity_controller/commands (wheel velocity)
            Node(
                package = "controller_manager",
                executable = "spawner",
                arguments = [
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ],
            ),
            
            # simple_controller - Node for spawning a simple_controller that use the JointGroupVelocityController and:
            # 1. subscribes to the bumperbot_control/cmd_vel topic (robot velocity) and 
            # 2. publishes on the simple_velocity_controller/commands (wheel velocity) 

            # The Python implementation of simple_controller if use_python == True
            Node(
                package = "bumperbot_controller",
                executable = "simple_controller.py",
                parameters = [{"wheel_radius": wheel_radius,
                            "wheel_separation": wheel_separation}],
                condition = IfCondition(use_python)
            ),

            # The C++ simple_controller executable if use_python == False
            Node(
                package = "bumperbot_controller",
                executable = "simple_controller",
                parameters = [{"wheel_radius": wheel_radius,
                            "wheel_separation": wheel_separation}],
                condition = UnlessCondition(use_python)
            )
        ]
    )
    



    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
    ])