# --- START OF FILE dev_vision.launch.py ---

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_pkg = get_package_share_directory('hexarover_bringup')

    arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Czy otworzyć RViz2?'
    )

    # 1. LIDAR (Bezpośrednie uruchomienie węzła z twardym przypisaniem unikalnego portu)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_8f08372c12a98349a38bcf6b24d6f895-if00-port0',
            'serial_baudrate': 256000,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity'
        }],
        output='screen'
    )

    # 2. VISION NODE – Zunifikowany YOLO + fuzja LiDAR + bezpośrednie pobieranie z kamery
    vision_node = Node(
        package='hexarover_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
    )

    # 3. FOLLOWER NODE – PID, subskrybuje /human_angle i /human_distance, publikuje /cmd_vel
    follower_node = Node(
        package='hexarover_vision',
        executable='follower_node',
        name='follower_node',
        output='screen',
    )

    # 4. CYTRON DRIVER – subskrybuje /cmd_vel, steruje silnikami przez UART/USB
    cytron_node = Node(
        package='cytron_driver',
        executable='cytron_node',
        name='cytron_node',
        output='screen',
    )

    # 5. STATIC TF: laser → lidar_link
    tf_laser_to_lidar_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_to_lidar_link',
        arguments=['0', '0', '0', '4.7124', '0', '0', 'laser', 'lidar_link'],
        output='screen',
    )

    # 6. RVIZ2
    rviz_config = os.path.join(bringup_pkg, 'rviz', 'vision_dev.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    return LaunchDescription([
        #arg_rviz,
        lidar_node,      # Bezpośredni, bezpieczny węzeł LiDAR
        vision_node,
        follower_node,  
        cytron_node,    
        tf_laser_to_lidar_link,
        #rviz_node,
    ])