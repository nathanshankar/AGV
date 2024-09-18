import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    ld = LaunchDescription()


    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'gazebo_worlds_sim'
    file_subpath = 'urdf/AGV_V1_gz.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('AGV_V1'), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/world.launch.py']),
        launch_arguments={}.items(),
    )
    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    node_spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', '/robot_description',
                                   '-z', '0.5'],
                        output='screen')

    # robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw}] # add other parameters here if required
    )
    
    # joint state publisher node
    node_joint_state_publisher = Node(
         package='joint_state_publisher',
         executable='joint_state_publisher',
         name='joint_state_publisher',
         )
    
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    #ld.add_action(declare_joy_vel)
    ld.add_action(launch_gazebo)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_joint_state_publisher)
    return ld