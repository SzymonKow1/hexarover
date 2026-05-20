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
    desc_pkg_name = 'hexarover_description'
    bringup_pkg_name = 'hexarover_bringup'

    # ARGUMENTY
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz2'
    )
    
    # 1. KONFIGURACJA MODELU
    pkg_path = get_package_share_directory(desc_pkg_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'hexarover.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    
    # WAŻNE NA SPRZĘCIE: use_sim_time musi być False!
    params = {'robot_description': doc.toxml(), 'use_sim_time': False}

    # 2. ROBOT STATE PUBLISHER (Mózg TFów)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # 3. JOINT STATE PUBLISHER
    # Zastępuje to, co wcześniej robił bridge z Gazebo (publikuje stan kół dla TF)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}]
    )

    # 4. RVIZ2
    rviz_config_file = os.path.join(
        get_package_share_directory(bringup_pkg_name), 'rviz', 'rviz.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': False}]
    )
    
    # 5. Laser Scan Matcher (Odometria z lasera)
    scan_matcher_config = os.path.join(get_package_share_directory(bringup_pkg_name), 'config', 'scan_matcher.yaml')
    
    scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[scan_matcher_config, {'use_sim_time': False}],
        remappings=[
            ('/scan', '/scan')
        ]
    )
    
    # 6. SLAM TOOLBOX
    slam_config_path = os.path.join(get_package_share_directory(bringup_pkg_name), 'config', 'slam_toolbox.yaml')
    
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_config_path,
            'use_sim_time': 'false' # Parametry startowe w stringu
        }.items()
    )
    
    # 7. NAV2
    pkg_bringup_path = get_package_share_directory(bringup_pkg_name)
    nav2_params_path = os.path.join(pkg_bringup_path, 'config', 'nav2.yaml')
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': 'false', # Parametry startowe w stringu
            'params_file': nav2_params_path,
            'map': '',
            'autostart': 'true'
        }.items()
    )
    
    delayed_nav2 = TimerAction(
        period=5.0, # Czekamy 5 sekund, by SLAM i odometria zdążyły wstać
        actions=[nav2_launch]
    )
     
    return LaunchDescription([
        rviz_arg,
        node_robot_state_publisher,
        joint_state_publisher_node,
        rviz_node,
        scan_matcher_node,
        slam_toolbox_launch,
        delayed_nav2
    ])