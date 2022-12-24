import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    slam_config_path = LaunchConfiguration('slam_config_path')
    rviz_config_path = LaunchConfiguration('rviz_config_path')

    declare_slam_config_path = DeclareLaunchArgument(
        'slam_config_path',
        default_value=os.path.join(get_package_share_directory('diff_bot_launch'),
                                   'config', 'diff_bot_slam_toolbox.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    declare_rviz_config_path = DeclareLaunchArgument(
        'rviz_config_path',
        default_value=os.path.join(get_package_share_directory('diff_bot_launch'),
                                   'rviz', 'diff_bot_slam_toolbox.rviz'),
        description='Rviz2 config for the slam_toolbox visualization')

    diff_bot_slam_dir = get_package_share_directory('diff_bot_slam')
    diff_bot_slam_launch_dir = os.path.join(diff_bot_slam_dir, 'launch')

    slam_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(diff_bot_slam_launch_dir,
                                                       'online_async_launch.py')),
            launch_arguments={'slam_params_file': slam_config_path}.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config_path],
            output='screen')
    ])

    ld = LaunchDescription()

    ld.add_action(declare_slam_config_path)
    ld.add_action(declare_rviz_config_path)
    ld.add_action(slam_group)

    return ld
