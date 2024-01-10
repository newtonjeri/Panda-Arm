import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():

    urdf_file = os.path.join(get_package_share_directory("panda_bot_description"), "urdf", "panda.xacro")

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

    # Environment Variable
    os.environ["GAZEBO_MODEL_PATH"] = os.path.join(get_package_prefix("panda_bot_description"), "share")


    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        )
    )

    robot_spawner = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "panda", "-topic", "robot_description"],
        output="screen"
    )


    return LaunchDescription(
        [
            model_arg,
            robot_state_publisher,
            gazebo,
            robot_spawner
        ]
    )