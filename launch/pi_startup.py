import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package= os.path.join(get_package_share_directory('myrt_robot'))
    #map_file = os.path.join(package,'maps','my_map.yaml')
    #nav2_param_file = os.path.join(package,'config','nav2_params.yaml')
    world_file= os.path.join(get_package_share_directory('myrt_robot'),'worlds','myrt_world.world')
    #rviz_config_dir = os.path.join(config_dir,'navigation.rviz')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('myrt_robot'),'/launch','/launch_sim.launch.py']),
            launch_arguments={
            'world':world_file}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('myrt_robot'),'/launch','/online_async_launch.py']),
            launch_arguments={
            'use_sim_time': 'true'}.items(),

        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([get_package_share_directory('myrt_robot'),'/launch','/navigation_launch.py']),
            launch_arguments={
            'use_sim_time':'true'}.items(),

        ),

    ])

    """ def button_callback():
            print("Button pressed!")

            navigate = Node(
            package="myrt_robot",
            executable="single_goal_nav")

            ld = LaunchDescription()
            ld.add_action(navigate)
            return ld
            

    def main():
        #BUTTON_PIN = 17
        #GPIO.setmode(GPIO.BCM)
        #GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        #GPIO.add_event_detect(BUTTON_PIN, GPIO.FALLING, callback=button_callback, bouncetime=300)
        generate_launch_description()
        
        # Execute main loop to handle navigation
        


    if __name__ == '__main__':
        main() """