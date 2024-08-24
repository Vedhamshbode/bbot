from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('bbot_new_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'bbot_new.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    world_file = os.path.join(share_dir, 'worlds', 'aruco.world')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(share_dir,'launch','rsp.launch.py')]), 
                launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            # 'pause': 'true'
            "world": world_file
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'bbot_new',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    diff_cont_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    return LaunchDescription([
        rsp,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        diff_cont_spawner,
        joint_broad_spawner
    ])
