import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    desc_pkg_name   = 'hexarover_description'
    bringup_pkg_name = 'hexarover_bringup'

    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false', description='Otworzyc RViz2?'
    )

    # 1. ROBOT DESCRIPTION
    pkg_path   = get_package_share_directory(desc_pkg_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'hexarover.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'use_sim_time': False}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 2. LIDAR
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/lidar',
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }],
        output='screen'
    )

    # 3. STATIC TF: laser -> lidar_link (offset 90 stopni)
    tf_laser_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_to_lidar_link',
        arguments=['0', '0', '0', '4.7124', '0', '0', 'laser', 'lidar_link'],
        output='screen',
    )

    # 4. SCAN MATCHER (odometria z lasera)
    scan_matcher_config = os.path.join(
        get_package_share_directory(bringup_pkg_name), 'config', 'scan_matcher.yaml'
    )
    scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[scan_matcher_config, {'use_sim_time': False}],
        remappings=[('/odom', '/odom_laser')]
    )

    # 5. EKF (fuzja odometrii)
    ekf_config = os.path.join(
        get_package_share_directory(bringup_pkg_name), 'config', 'ekf.yaml'
    )
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': False}]
    )

    # 6. SLAM TOOLBOX
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

    # 7. NAV2 (z opóźnieniem 10s żeby SLAM zdążył wstać)
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
    delayed_nav2 = TimerAction(period=10.0, actions=[nav2_launch])

    # 8. VISION NODE
    vision_node = Node(
        package='hexarover_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 9. FOLLOWER NODE
    follower_node = Node(
        package='hexarover_vision',
        executable='follower_node',
        name='follower_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 10. CYTRON DRIVER
    cytron_node = Node(
        package='cytron_driver',
        executable='cytron_node',
        name='cytron_node',
        output='screen',
    )

    # 11. RVIZ2 (domyslnie wylaczony - uruchamiamy na laptopie)
    rviz_config = os.path.join(
        get_package_share_directory(bringup_pkg_name), 'rviz', 'rviz.rviz'
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        lidar_node,
        tf_laser_to_lidar,
        scan_matcher_node,
        ekf_node,
        slam_toolbox,
        delayed_nav2,
        vision_node,
        follower_node,
        cytron_node,
        rviz_node,
    ])
