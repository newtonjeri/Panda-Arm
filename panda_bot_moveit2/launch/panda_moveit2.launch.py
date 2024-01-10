import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    is_sim = LaunchConfiguration("is_sim")

    is_sim_arg = DeclareLaunchArgument(
        "is_sim",
        default_value = 'True'
    )

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name = "panda_bot_moveit2")
        .robot_description(
            os.path.join(get_package_share_directory("panda_bot_description"), "urdf", "panda.xacro")
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package = "moveit_ros_move_group",
        executable = "move_group",
        output = "screen",
        parameters = [
            moveit_config.to_dict(),
            {"use_sim_time": is_sim},
            {"publish_robot_description_semantic": True}
        ],
        arguments = ["--ros-args", "--log-level", "info"],
    
    )

    # RViz Configuratin
    rviz_config = os.path.join(
        get_package_share_directory("panda_bot_moveit2"),
        "config",
        "moveit.rviz",
    )

    # Rviz2 node
    rviz_node = Node(
        package = "rviz2",
        executable = "rviz2",
        name = "rviz2",
        output = "log",
        arguments=["-d", rviz_config],
        parameters = [
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ]
    )

    return LaunchDescription([
        is_sim_arg,
        move_group_node,
        rviz_node
    ])