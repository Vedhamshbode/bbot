from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('bbot_new_description'),'launch','rsp.launch.py')]), 
                launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'true'}.items()
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("bbot_new_description"), "urdf", "bbot_new.xacro"]
            ),
        ]
    )

    robot_description = {"robot_description": robot_description_content}

    controller_params_file = os.path.join(get_package_share_directory("bbot_new_description"), 'config', 'my_controllers.yaml')
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("bbot_new_description"), "config", "hardware_navigation.rviz"]
    )

    # robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_decription'])

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controller_params_file]
    )
    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    diff_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
    )

    # Delay rviz start after `joint_state_broadcaster`
    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_diff_cont_spawner= RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_cont_spawner],
        )
    )

    delay_joint_state_broadcaster_spawner= RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    nodes = [
        rsp,
        delayed_controller_manager,
        delay_diff_cont_spawner,
        delay_joint_state_broadcaster_spawner
    ]

    return LaunchDescription(nodes)
