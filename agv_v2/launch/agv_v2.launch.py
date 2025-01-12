import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch.actions import TimerAction


def generate_launch_description():
    ld = LaunchDescription()


    # Specify the name of the package and path to xacro file within the package
    sim_pkg_name = 'gazebo_worlds_sim'
    urdf_file_subpath = 'urdf/agv_v1.urdf.xacro'

    # Use xacro to process the file
    xacro_file = os.path.join(get_package_share_directory('agv_v1'), urdf_file_subpath)
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

    cmd_vel_params_file = PathJoinSubstitution([
        get_package_share_directory('agv_v2'), 'config', 'twist_mux_params.yaml'
    ])

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[cmd_vel_params_file],
        remappings={('/cmd_vel_out', '/agv/cmd_vel')},
    )

    # Bridge
    # https://github.com/gazebosim/ros_gz/tree/humble/ros_gz_bridge
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=  [
                    '/clock'                           + '@rosgraph_msgs/msg/Clock'   + '[' + 'gz.msgs.Clock',
                    '/model/agv/cmd_vel'  + '@geometry_msgs/msg/Twist'   + '@' + 'gz.msgs.Twist',
                    '/model/agv/odometry' + '@nav_msgs/msg/Odometry'     + '[' + 'gz.msgs.Odometry',
                    '/model/agv/scan'     + '@sensor_msgs/msg/LaserScan' + '[' + 'gz.msgs.LaserScan',
                    '/model/agv/tf'       + '@tf2_msgs/msg/TFMessage' + '[' + 'gz.msgs.Pose_V',
                    '/model/agv/imu'      + '@sensor_msgs/msg/Imu'       + '[' + 'gz.msgs.IMU',
                    '/world/empty/model/agv/joint_state' + '@sensor_msgs/msg/JointState' + '[' + 'gz.msgs.Model',
                    '/world/empty/model/agv/link/gripper_base/sensor/color/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[' + 'gz.msgs.Camera',
                    '/world/empty/model/agv/link/gripper_base/sensor/color/image' + '@sensor_msgs/msg/Image' + '[' + 'gz.msgs.Image',
                    '/world/empty/model/agv/link/gripper_base/sensor/depth/camera_info' + '@sensor_msgs/msg/CameraInfo' + '[' + 'gz.msgs.Camera',
                    '/world/empty/model/agv/link/gripper_base/sensor/depth/depth_image' + '@sensor_msgs/msg/Image' + '[' + 'gz.msgs.Image',
                    '/world/empty/model/agv/link/gripper_base/sensor/depth/depth_image/points' + '@sensor_msgs/msg/PointCloud2' + '[' + 'gz.msgs.PointCloudPacked',
                    ],
        parameters= [{'qos_overrides./agv_v1.subscriber.reliability': 'reliable'},{'qos_overrides./agv_v1.subscriber.durability': 'transient_local'}],
        remappings= [
                    ('/model/agv/cmd_vel',  '/agv/cmd_vel'),
                    ('/model/agv/odometry', '/odom'   ),
                    ('/model/agv/scan',     '/scan'   ),
                    ('/model/agv/tf',       '/tf'     ),
                    ('/model/agv/imu',      '/imu_raw'),
                    ('/world/empty/model/agv/joint_state', 'joint_states'),
                    ('/world/empty/model/agv/link/gripper_base/sensor/color/camera_info', 'camera_info'),
                    ('/world/empty/model/agv/link/gripper_base/sensor/color/image', 'image_raw'),
                    ('/world/empty/model/agv/link/gripper_base/sensor/depth/camera_info', 'depth_camera_info'),
                    ('/world/empty/model/agv/link/gripper_base/sensor/depth/depth_image', 'depth_image'),
                    ('/world/empty/model/agv/link/gripper_base/sensor/depth/depth_image/points', '/agv/points'),
                    ],
        output='screen'
    )

    # Rviz node
    node_rviz = Node(
        package='rviz2',
        namespace='',
        executable='rviz2',
        name='rviz2',
        arguments=['-d' + os.path.join(get_package_share_directory('agv_v1'), 'rviz', 'agv_v1_view.rviz')]
    )
    
    
    # joint state publisher node
    node_joint_state_publisher = Node(
         package='joint_state_publisher',
         executable='joint_state_publisher',
         name='joint_state_publisher',
         )
    
    lidar_static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'lidar_link', 'agv/base_link/generic_lidar_sensor'],
        output='screen'
    )

    node_controller_manager = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_manager',
        arguments=['arm_controller', 'joint_state_broadcaster'],
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    arm_home_trajectory_publisher = Node(
        package='agv_v2',
        executable='arm_trajectory_publisher.py',
        name='arm_trajectory_publisher',
        output='screen'
    )

    delay_action = TimerAction(
        period=15.0,
        actions=[arm_home_trajectory_publisher]
    )

    depth_pcl_remap = Node(
        package='agv_v2',
        executable='agv_pcl_remap',
        name='agv_pcl_remap',
        output='screen'
    )
    
    
    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=True))
    ld.add_action(launch_gazebo)
    ld.add_action(node_spawn_entity)
    ld.add_action(node_robot_state_publisher)
    ld.add_action(node_ros_gz_bridge)
    ld.add_action(teleop_twist_keyboard_node)
    ld.add_action(lidar_static_tf_publisher)
    # ld.add_action(node_joint_state_publisher)
    ld.add_action(twist_mux_node)
    ld.add_action(node_controller_manager)
    ld.add_action(depth_pcl_remap)
    ld.add_action(node_rviz)
    ld.add_action(delay_action)
    
    return ld