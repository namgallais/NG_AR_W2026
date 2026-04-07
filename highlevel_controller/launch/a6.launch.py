from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    control_orientation = LaunchConfiguration('control_orientation')

    kinematic_pkg = get_package_share_directory('kinematic_controller')

    launch_3d = os.path.join(kinematic_pkg, 'launch', '3d.launch.xml')
    launch_6d = os.path.join(kinematic_pkg, 'launch', '6d.launch.xml')

    return LaunchDescription([

        DeclareLaunchArgument(
            'control_orientation',
            default_value='false',
            description='Switch between 3D and 6D control'
        ),

        # 3D (default)
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(launch_3d),
            condition=UnlessCondition(control_orientation)
        ),

        # 6D
        IncludeLaunchDescription(
            XMLLaunchDescriptionSource(launch_6d),
            condition=IfCondition(control_orientation)
        ),
    ])
