import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    urdf_file = os.path.join(get_package_share_directory("panda_bot_description"), "urdf", "panda.xacro")

    # with open(urdf_file, 'r') as infp:
    #     robot_description = infp.read()


    model_arg = DeclareLaunchArgument(
        "model",
        default_value = urdf_file,
        description = "Absolute path to robot urdf file"
    )

    robot_description = ParameterValue(
        Command(
            ["xacro ", LaunchConfiguration("model")]
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable = "robot_state_publisher",
        name = "robot_state_publisher",
        output = "screen",
        parameters=[{"robot_description": robot_description}],

    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable = "joint_state_publisher_gui",
        output = "screen"
    )

    rviz_config = os.path.join(get_package_share_directory("panda_bot_description"), "rviz", "panda_rviz.rviz")

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name = "rviz2",
        output = "screen",
        arguments=["-d", rviz_config]

    )

    return LaunchDescription([
        model_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])