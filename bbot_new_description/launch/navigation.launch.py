from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory



def generate_launch_description():

    share_dir = get_package_share_directory('bbot_new_description')

    mapper_params_file = os.path.join(share_dir, 'config', 'mapper_params_online_async.yaml')

    rviz2_config_file = os.path.join(share_dir, 'config', 'navigationrviz.rviz')


    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bbot_new_description'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
    )

    slamtoolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ]),
        launch_arguments={
            'slam_params_file': mapper_params_file,
            'use_sim_time': 'true'
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', rviz2_config_file
        ]
    )

    navigation_launch = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py', 'use_sim_time:=true'],
        output='screen'
    )

    teleop_twist_keyboard_node = ExecuteProcess(
        cmd=['gnome-terminal', '--', 'ros2', 'run', 'teleop_twist_keyboard', 'teleop_twist_keyboard'],
        output='screen'
    )

    aruco_tf_publisher_node=Node(
        package='bbot_new_description',
        executable='aruco_tf_publisher'
    )

    return LaunchDescription([
        gazebo_launch,
        slamtoolbox_launch,
        rviz2_node,
        TimerAction(
            period=5.0,
            actions=[navigation_launch]
        ),
        teleop_twist_keyboard_node,
        aruco_tf_publisher_node
    ])

