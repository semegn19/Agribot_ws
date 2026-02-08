import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. Paths
    pkg_gazebo = get_package_share_directory('agribot_gazebo')
    pkg_description = get_package_share_directory('agribot_description')
    world_file = os.path.join(pkg_gazebo, 'worlds', 'ethiopian_farm.world')
    xacro_file = os.path.join(pkg_description, 'urdf', 'agribot.urdf.xacro')

    # 2. Process URDF
    robot_description_config = xacro.process_file(xacro_file)
    params = {'robot_description': robot_description_config.toxml()}

    # 3. Nodes
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config.toxml(),
            'use_sim_time': True, 
            'publish_frequency': 30.0,
        }]
    )

    # 4. Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 5. Spawn Robot Node
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'agribot', '-z', '0.5'],
        output='screen',
    )

    return LaunchDescription([
        node_robot_state_publisher,
        gazebo,
        spawn_robot
    ])
