import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('myrt_robot'),'config')
    map_file = os.path.join(get_package_share_directory('myrt_robot'),'maps','my_map.yaml')
    param_file = os.path.join(config_dir,'nav3_params.yaml')
    world_file= os.path.join(get_package_share_directory('myrt_robot'),'worlds','myrt_world.world')
    #rviz_config_dir = os.path.join(config_dir,'navigation.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('myrt_robot'),'/launch','/launch_sim.launch.py']),
            launch_arguments={
            'world':world_file}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('nav2_bringup'),'/launch','/bringup_launch.py']),
            launch_arguments={
            'map':map_file,
            'params_file': param_file}.items(),

        ),

    ])

    """  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_node',
            arguments=['-d', rviz_config_dir],
            output='screen'

            ), """