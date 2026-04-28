from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown

def generate_launch_description():
    kortex_sim_pkg = get_package_share_directory('kortex_sim')
    kinematic_pkg = get_package_share_directory('kinematic_controller')
    pick_n_place_pkg = get_package_share_directory('pick_n_place')
    
    
    launch_kortex      = os.path.join(kortex_sim_pkg, 'launch', 'final.launch.py')
    launch_6d         = os.path.join(kinematic_pkg, 'launch', '6d.launch.xml')
    launch_pick_place = os.path.join(pick_n_place_pkg,  'launch', 'pick_n_place.launch.xml')

    return LaunchDescription([
        # Kortex sim
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_kortex)
        ),
    
        # 6D kinematic — starts after 5s
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(launch_6d)
                ),
            ]
        ),

        # Pick and place — starts after 5s + 10s = 15s
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    XMLLaunchDescriptionSource(launch_pick_place)
                ),
            ]
        ),
        
        
        # Kill gazebo cleanly on shutdown
        RegisterEventHandler(
            OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['pkill', '-f', 'gzserver'],
                    output='screen'
                )
            ]
        )
),
    ])
