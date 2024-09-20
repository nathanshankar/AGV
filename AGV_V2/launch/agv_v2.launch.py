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
    sim_pkg_name = 'gazebo_worlds_sim'
    urdf_file_subpath = 'urdf/AGV_V1.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('AGV_V1'), urdf_file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    launch_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(sim_pkg_name), '/launch', '/world.launch.py']),
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

    teleop_twist_keyboard_node = Node(
            package='teleop_twist_keyboard', executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node', 
            remappings={('/cmd_vel', '/teleop_cmd_vel')},
            output='screen',
            prefix = 'xterm -e',
            )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[
            {
                'output_topic': '/leo/cmd_vel',
                'topics': {
                    'keyboard': {
                        'topic': '/teleop_cmd_vel',
                        'timeout': 0.5,
                        'priority': 15
                    },
                    'locks': {
                        'auto_nav': {
                            'topic': '/nav_cmd_vel',
                            'timeout': 0.0,
                            'priority': 10
                        }
                    }
                }
            }
        ],
        remappings={('/cmd_vel_out', '/leo/cmd_vel')},
    )
    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'ignition.msgs.Clock',
                    '/model/leo/cmd_vel'  + '@geometry_msgs/msg/Twist'   + '@' + 'ignition.msgs.Twist',
                    '/model/leo/odometry' + '@nav_msgs/msg/Odometry'     + '[' + 'ignition.msgs.Odometry',
                    '/model/leo/scan'     + '@sensor_msgs/msg/LaserScan' + '[' + 'ignition.msgs.LaserScan',
                    '/model/leo/tf'       + '@tf2_msgs/msg/TFMessage' + '[' + 'ignition.msgs.Pose_V',
                    '/model/leo/imu'      + '@sensor_msgs/msg/Imu'       + '[' + 'ignition.msgs.IMU',
                    '/world/empty/model/leo/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'ignition.msgs.Model',
                    ],
        parameters= [{'qos_overrides./leo_v1.subscriber.reliability': 'reliable'},{'qos_overrides./leo_v1.subscriber.durability': 'transient_local'}],
        remappings= [
                    ('/model/leo/cmd_vel',  '/leo/cmd_vel'),
                    ('/model/leo/odometry', '/odom'   ),
                    ('/model/leo/scan',     '/scan'   ),
                    ('/model/leo/tf',       '/tf'     ),
                    ('/model/leo/imu',      '/imu_raw'),
                    ('/world/empty/model/leo/joint_state', 'joint_states'),
                    ],
        output='screen'
    )

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('AGV_V1'), 'rviz', 'AGV_V1_view.rviz')]
    )
    
    
    # joint state publisher node
    node_joint_state_publisher = Node(
         package='joint_state_publisher',
         executable='joint_state_publisher',
         name='joint_state_publisher',
         )
    
    
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_gazebo)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(teleop_twist_keyboard_node)
    # ld.add_action(node_joint_state_publisher)
    ld.add_action(twist_mux_node)
    ld.add_action(node_rviz)
    return ld