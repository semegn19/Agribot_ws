#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    pkg_this = get_package_share_directory('agribot_nav2_bringup')

    params_file = LaunchConfiguration('params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    use_sim_time_bool = use_sim_time.lower() == 'true'
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic').perform(context)

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav2, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': str(use_sim_time_bool),
            'map': map_yaml,
            'cmd_vel_topic': '/model/agribot/cmd_vel',
            'cmd_vel_remap': '/model/agribot/cmd_vel',
        }.items()
    )

    return [bringup]

def generate_launch_description():
    pkg_this = get_package_share_directory('agribot_nav2_bringup')
    default_params = os.path.join(pkg_this, 'params', 'nav2_params.yaml')
    default_map = ''  
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Full path to map yaml file to load (optional). If blank nav2 will not start map_server.'
    )
    declare_cmd_vel = DeclareLaunchArgument(
    	'cmd_vel_topic',
   	 default_value='/model/agribot/cmd_vel',
  	  description='cmd_vel topic to publish velocity 	commands'
    )


    return LaunchDescription([
        declare_params,
        declare_sim_time,
        declare_map,
        declare_cmd_vel,
        OpaqueFunction(function=launch_setup)
    ])

