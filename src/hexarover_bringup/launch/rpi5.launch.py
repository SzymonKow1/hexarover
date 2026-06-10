import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    desc_pkg_name   = 'hexarover_description'
    bringup_pkg_name = 'hexarover_bringup'

    # 1. ROBOT DESCRIPTION (Ważne, by TF były publikowane stąd)
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

    # 3. STATIC TF: laser -> lidar_link
    tf_laser_to_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_to_lidar_link',
        arguments=['0', '0', '0', '4.7124', '0', '0', 'laser', 'lidar_link'],
        output='screen',
    )

    # 4. SCAN MATCHER (Odometria liczona na szybkim CPU RPi 5)
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

    # 5. EKF (Fuzja odometrii na RPi 5)
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

    # 6. VISION & YOLO
    vision_node = Node(
        package='hexarover_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 7. FOLLOWER (pamiętaj o zmianie tematu cmd_vel w kodzie!)
    follower_node = Node(
        package='hexarover_vision',
        executable='follower_node',
        name='follower_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # 8. CYTRON DRIVER
    cytron_node = Node(
        package='cytron_driver',
        executable='cytron_node',
        name='cytron_node',
        output='screen',
    )
    # Ścieżka do pliku konfiguracyjnego
    twist_mux_config = os.path.join(
        get_package_share_directory(bringup_pkg_name), 'config', 'twist_mux.yaml'
    )

    # Węzeł twist_mux
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[twist_mux_config],
        # twist_mux domyślnie wypluwa dane na /cmd_vel_out, 
        # przemapowujemy to na /cmd_vel, którego słucha Wasz Cytron
        remappings=[('/cmd_vel_out', '/cmd_vel')]
    )
    return LaunchDescription([
        robot_state_publisher,
        lidar_node,
        tf_laser_to_lidar,
        scan_matcher_node,
        ekf_node,
        vision_node,
        follower_node,
        cytron_node,
        twist_mux_node
    ])