from launch import LaunchDescription
from launch import actions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import launch
import launch_xml
import os

def generate_launch_description():

    urdf_file = PathJoinSubstitution([
        FindPackageShare('kortex_description'),
        'robots',
        'gen3.urdf'
    ])

    # Use os.path for YAML config — parameters[] needs a resolved string, not a substitution
    config_path = os.path.join(
        get_package_share_directory('dynamic_controller'),
        'config',
        'without_redundancy.yaml'
    )

    potential_field_task_path = [
        FindPackageShare('potential_field'),
        '/launch/potential_field_task.launch.xml'
    ]
    
    potential_field_joint_path = [
        FindPackageShare('potential_field'),
        '/launch/potential_field_joint.launch.xml'
    ]

    dynamic_controller_node = Node(
        package='dynamic_controller',
        executable='dynamic_controller',
        name='dynamic_controller',
        output='screen',
        parameters=[
            {'urdf_file_name': urdf_file},
            {'use_sim_time': True},
            config_path
        ]
    )

    
    potential_field_task = launch.actions.IncludeLaunchDescription(
        launch_xml.launch_description_sources.XMLLaunchDescriptionSource(
            potential_field_task_path
        )
    )

    potential_field_joint = launch.actions.IncludeLaunchDescription(
        launch_xml.launch_description_sources.XMLLaunchDescriptionSource(
            potential_field_joint_path
        )
    )


    ld = LaunchDescription()
    ld.add_action(dynamic_controller_node)
    ld.add_action(potential_field_task)
    ld.add_action(potential_field_joint)
    return ld
