from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    kortex_description_path = get_package_share_directory('kortex_description')
    dynamic_controller_path = get_package_share_directory('dynamic_controller')
    potential_field_path = get_package_share_directory('potential_field')

    urdf_file = os.path.join(kortex_description_path, 'robots', 'gen3.urdf')
    config_file = os.path.join(dynamic_controller_path, 'config', 'with_redundancy.yaml')
    potential_field_launch = os.path.join(potential_field_path, 'launch', 'potential_field_task.launch.xml')


    dynamic_controller_node = Node(
        package='dynamic_controller',
        executable='dynamic_controller',
        name='dynamic_controller',
        output='screen',
        parameters=[
            {'urdf_file_name': urdf_file},
            {'use_sim_time': True},
            config_file
        ]
    )


    potential_field = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(potential_field_launch)
    )


    return LaunchDescription([
        dynamic_controller_node,
        potential_field
    ])
