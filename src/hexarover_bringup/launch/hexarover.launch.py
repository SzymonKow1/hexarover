import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    desc_pkg_name = 'hexarover_description'
    bringup_pkg_name = 'hexarover_bringup'
    gazebo_pkg_name = 'hexarover_gazebo'

    # ARGUMENTY
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true', description='Open RViz2'
    )
    
    # 1. KONFIGURACJA MODELU
    pkg_path = get_package_share_directory(desc_pkg_name)
    xacro_file = os.path.join(pkg_path, 'urdf', 'hexarover.urdf.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    
    # WAŻNE: use_sim_time mówi ROSowi, żeby używał zegara Gazebo
    params = {'robot_description': doc.toxml(), 'use_sim_time': True}

    # 2. KONFIGURACJA SWIATA
    world_pkg_path = get_package_share_directory(gazebo_pkg_name)
    world_file = os.path.join(world_pkg_path, 'worlds', 'obstacles.sdf')

    # Uruchomienie Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 3. SPAWN ROBOTA
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'hexarover', '-z', '0.5'],
        output='screen'
    )

    # 4. ROBOT STATE PUBLISHER (Mózg TFów)
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params] # Tu przekazujemy use_sim_time
    )

    # 5. MOST (BRIDGE)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/joint_states@sensor_msgs/msg/JointState@gz.msgs.Model', # <--- NOWOŚĆ: Stan kół
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 6. RVIZ2
    rviz_config_file = os.path.join(
        get_package_share_directory(bringup_pkg_name), 'rviz', 'rviz.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(LaunchConfiguration('rviz')),
        parameters=[{'use_sim_time': True}] # RViz też musi znać czas symulacji
    )

    return LaunchDescription([
        rviz_arg,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        rviz_node
    ])