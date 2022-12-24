import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    diff_bot_control_dir = get_package_share_directory('diff_bot_control')
    diff_bot_control_launch_dir = os.path.join(
        diff_bot_control_dir, 'launch')

    diff_bot_launch_dir = get_package_share_directory('diff_bot_launch')
    diff_bot_launch_launch_dir = os.path.join(diff_bot_launch_dir, 'launch')

    sllidar_ros2_dir = get_package_share_directory('sllidar_ros2')
    sllidar_ros2_launch_dir = os.path.join(sllidar_ros2_dir, 'launch')

    bringup_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(diff_bot_control_launch_dir, 'diff_bot_control.launch.py')),
            launch_arguments={'open_rviz': 'false'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(diff_bot_launch_launch_dir, 'diff_bot_joy.launch.py')),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sllidar_ros2_launch_dir,
                                                       'sllidar_s1_launch.py')),
            launch_arguments={'frame_id': 'lidar_link'}.items()
        ),
    ])

    ld = LaunchDescription()

    ld.add_action(bringup_group)

    return ld
