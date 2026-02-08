#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_description = get_package_share_directory('agribot_description')
    pkg_agribot_navigation = get_package_share_directory('agribot_navigation')
    pkg_nav2_bringup = get_package_share_directory('agribot_nav2_bringup')
    map_path = os.path.join(pkg_nav2_bringup, 'maps', 'my_farm_map.yaml')

    # 1) spawn robot (gazebo + robot)
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'agribot_spawn.launch.py')
        )
    )

    # 2) ros_gz_bridge (include TF topics)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
        # Format: <gz_topic>@<ros_msg_type>@<gz_msg_type>
              '/model/agribot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist', 
              '/model/agribot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry', 
              '/model/agribot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
              #'tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V', 
              '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan', 
              '/camera@sensor_msgs/msg/Image@gz.msgs.Image', 
              '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model', 
              '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',		
        ],
        remappings=[
            ('/model/agribot/cmd_vel', '/cmd_vel'),
            ('/model/agribot/odometry', '/odom'),
            ('/model/agribot/tf', '/tf'), 
            #('/model/agribot/tf_static', '/tf_static'),
        ],
        parameters=[{
        'use_sim_time': True,
        'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )
   

    # 3) vision node
    vision_node = Node(
        package='agribot_vision',
        executable='detect_disease',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4) safety/mediator node
    nav_node = Node(
        package='agribot_navigation',
        executable='safety_stop',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )


    # 5) Nav2 bringup include (use the params from agribot_nav2_bringup)
    nav2_params = os.path.join(pkg_nav2_bringup, 'params', 'nav2_params.yaml')
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params,
            'autostart': 'True',
            'use_lifecycle_mgr': 'True',
            'slam': 'False',  #	True
            'use_sim_time': 'True',
            'map': map_path #LaunchConfiguration('map'), 
            # pass empty map or put a real map path here
            
        }.items()
    )

    # Delay Nav2 a few seconds so bridge & robot spawn finish
    delayed_nav2 = TimerAction(period=3.0, actions=[nav2_bringup])

    return LaunchDescription([
        spawn_robot,
        bridge,
        vision_node,
        delayed_nav2,
        nav_node,
    ])

