import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
import xacro


def generate_launch_description():
    ld = LaunchDescription()
    sdf_path = os.path.join(get_package_share_directory('gazebo_worlds_sim'), 'worlds', 'maze1.sdf')

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        gz_world_path =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + os.pathsep + os.path.join(get_package_share_directory('gazebo_worlds_sim'), "worlds")
    else:
        gz_world_path =  os.path.join(get_package_share_directory('gazebo_worlds_sim'), "worlds")

    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',value=[gz_world_path])
    


    # Launch world
    gz_start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
        launch_arguments={
            'gz_args' : '-r ' + 'empty.sdf'
            }.items(),
    )

    # Add features
    gz_spawn_objects = Node(package='ros_gz_sim', executable='create',
    arguments=['-file', sdf_path,
    '-x', '2.0',
    '-y', '0.5',
    '-z', '0.0'],
    output='screen'
    )

    # Add actions
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(ign_resource_path)
    ld.add_action(gz_spawn_objects)
    ld.add_action(gz_start_world)

    return ld