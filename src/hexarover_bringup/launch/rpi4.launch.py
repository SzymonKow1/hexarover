import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    bringup_pkg_name = 'hexarover_bringup'

    # 1. SLAM TOOLBOX (Wykorzystuje 8GB RAM RPi 4)
    slam_config = os.path.join(
        get_package_share_directory(bringup_pkg_name), 'config', 'slam_toolbox.yaml'
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config,
            'use_sim_time': 'false'
        }.items()
    )

    # 2. NAV2 (Nawigacja i Costmapy)
    nav2_params = os.path.join(
        get_package_share_directory(bringup_pkg_name), 'config', 'nav2.yaml'
    )
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': nav2_params,
        }.items()
    )
    
    # Opóźniamy Nav2 o 10 sekund, aby upewnić się, że SLAM odpalił i publikuje mapę
    delayed_nav2 = TimerAction(period=10.0, actions=[nav2_launch])

    return LaunchDescription([
        slam_toolbox,
        delayed_nav2
    ])