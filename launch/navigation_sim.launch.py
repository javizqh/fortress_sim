import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def generate_launch_description():
    package_dir = get_package_share_directory('fortress_sim')
    nav2_dir = get_package_share_directory('nav2_bringup')

    # Configuration Variables
    slam = LaunchConfiguration('slam')
    rviz = LaunchConfiguration('rviz')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_slam_cmd = DeclareLaunchArgument(
        'slam', default_value='False')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='True')

    declare_map_cmd = DeclareLaunchArgument(
        'map', default_value=os.path.join(
            package_dir,
            'maps',
            'aws_house.yaml')
    )

    declare_nav_params_cmd = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(
            package_dir,
            'params',
            'waffle_sim_nav_params.yaml')
    )

    # Actions
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': slam,
            'map': map_file,
            'params_file': params_file
        }.items()
    )

    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': rviz
        }.items()
    )


    ld = LaunchDescription()
    ld.add_action(declare_slam_cmd)
    ld.add_action(declare_nav_params_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(rviz_cmd)
    cmd_vel_remap = SetRemap(src='cmd_vel_nav', dst='cmd_vel')
    ld.add_action(cmd_vel_remap)

    return ld
