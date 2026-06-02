# --- START OF FILE dev_vision.launch.py ---

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    bringup_pkg = get_package_share_directory('hexarover_bringup')
    sllidar_pkg = get_package_share_directory('sllidar_ros2')

    arg_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Czy otworzyć RViz2?'
    )

    # 1. LIDAR (Poprawiono literówkę spacji w sllidar_a2m12_launch.py)
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_pkg, 'launch', 'sllidar_a2m12_launch.py')
        ),
        launch_arguments={'serial_port': '/dev/lidar'}.items()
    )


    # 3. VISION NODE – Zunifikowany YOLO + fuzja LiDAR + Kamera
    vision_node = Node(
        package='hexarover_vision',
        executable='vision_node',
        name='vision_node',
        output='screen',
    )

    # 4. STATIC TF: laser → lidar_link
    tf_laser_to_lidar_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_laser_to_lidar_link',
        arguments=['0', '0', '0', '4.7124', '0', '0', 'laser', 'lidar_link'],
        output='screen',
    )

    # 5. RVIZ2
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
        arg_rviz,
        lidar_launch,
        vision_node,
        tf_laser_to_lidar_link,
        rviz_node,
    ])