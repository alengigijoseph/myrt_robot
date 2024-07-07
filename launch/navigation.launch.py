import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    package_name= 'myrt_robot'

    use_sim_time = LaunchConfiguration('use_sim_time')

    config_dir = os.path.join(get_package_share_directory(package_name),'config')
    map_file = os.path.join(get_package_share_directory(package_name),'maps','my_map.yaml')
    param_file = os.path.join(config_dir,'nav2_params.yaml')
    #rviz_config_dir = os.path.join(config_dir,'navigation.rviz')

    return LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
            launch_arguments={
            'map':map_file,
            'params_file': param_file,
            'use_sim_time':use_sim_time}.items(),
            

        )
    ])