from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    joint_trajectory_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "panda_arm_controller",
                "--controller-manager", "/controller_manager",
            ]

    )

    joint_state_broadcaster = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "joint_state_broadcaster",
                "--controller-manager", "/controller_manager",
            ]
        )
    
    gripper_controller = Node(
            package="controller_manager",
            executable="spawner",
            arguments=[
                "panda_gripper_controller",
                "--controller-manager", "/controller_manager",
            ]
    )

    return LaunchDescription(
        [
            joint_state_broadcaster,
            joint_trajectory_controller,
            gripper_controller, 
        ]
    )
