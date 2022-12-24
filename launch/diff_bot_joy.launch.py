import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, GroupAction, RegisterEventHandler

from launch_ros.actions import Node


def generate_launch_description():
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration(
        'config_filepath')

    joy = GroupAction(
        actions=[
            DeclareLaunchArgument(
                'joy_vel', default_value='cmd_vel'),
            DeclareLaunchArgument(
                'joy_config', default_value='ps3'),
            DeclareLaunchArgument(
                'joy_dev', default_value='/dev/input/js0'),
            DeclareLaunchArgument('config_filepath', default_value=[
                TextSubstitution(text=os.path.join(
                    get_package_share_directory('diff_bot_launch'), 'config', '')),
                'xbox', TextSubstitution(text='.config.yaml')]),

            Node(
                package='joy', executable='joy_node', name='joy_node',
                parameters=[{
                    'dev': joy_dev,
                    'deadzone': 0.3,
                    'autorepeat_rate': 20.0,
                }]),
            Node(
                package='teleop_twist_joy', executable='teleop_node',
                name='teleop_twist_joy_node', parameters=[config_filepath],
                remappings={
                    ('/cmd_vel', LaunchConfiguration('joy_vel'))},
            ),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(joy)

    return ld
