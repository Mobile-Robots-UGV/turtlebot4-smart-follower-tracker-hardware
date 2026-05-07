import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('sft_hardware_tracker')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    pkg_irobot_create_description = get_package_share_directory('irobot_create_description')
    pkg_irobot_create_control = get_package_share_directory('irobot_create_control')
    pkg_nav2_minimal_tb4_description = get_package_share_directory('nav2_minimal_tb4_description')
    pkg_board_pose_ros = get_package_share_directory('board_pose_ros')

    camera_calibration_file = os.path.join(pkg_board_pose_ros, 'config', 'camera_calib_oak.npz')
    sim_board_config_file = os.path.join(pkg_share, 'config', 'sim_board_config.json')
    config_file = os.path.join(pkg_share, 'config', 'sft_hardware_recovery.yaml')
    control_config_file = os.path.join(pkg_irobot_create_control, 'config', 'control.yaml')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'sft_turtlebot_hardware.rviz')

    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    world = LaunchConfiguration('world')
    turtlebot_model = LaunchConfiguration('turtlebot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_board_pose = LaunchConfiguration('start_board_pose')
    start_tracker = LaunchConfiguration('start_tracker')
    start_follower = LaunchConfiguration('start_follower')
    spawn_leader = LaunchConfiguration('spawn_leader')
    leader_name = LaunchConfiguration('leader_name')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    turtlebot_xacro = PathJoinSubstitution([
        FindPackageShare('turtlebot4_description'),
        'urdf',
        turtlebot_model,
        'turtlebot4.urdf.xacro',
    ])
    leader_xacro = PathJoinSubstitution([
        FindPackageShare('sft_hardware_tracker'),
        'urdf',
        'leader_turtlebot4_aruco.urdf.xacro',
    ])
    world_sdf = PathJoinSubstitution([
        FindPackageShare('sft_hardware_tracker'),
        'worlds',
        [world, '.sdf'],
    ])
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    gz_resource_paths = [
        str(Path(pkg_share).parent.resolve()),
        os.path.join(pkg_share, 'worlds'),
        str(Path(pkg_turtlebot4_description).parent.resolve()),
        str(Path(pkg_irobot_create_description).parent.resolve()),
        str(Path(pkg_nav2_minimal_tb4_description).parent.resolve()),
    ]
    existing_gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH')
    if existing_gz_resource_path:
        gz_resource_paths.append(existing_gz_resource_path)

    robot_description = ParameterValue(
        Command([
            'xacro', ' ', turtlebot_xacro, ' ',
            'gazebo:=ignition', ' ',
            'namespace:=', namespace,
        ]),
        value_type=str,
    )
    leader_description = ParameterValue(
        Command([
            'xacro', ' ', leader_xacro, ' ',
            'gazebo:=ignition',
        ]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments={
            'gz_args': [world_sdf, ' -r -v 2'],
        }.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_prefix': ParameterValue([namespace, '/'], value_type=str)},
            {'robot_description': robot_description},
        ],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    leader_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='leader',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_prefix': 'leader/'},
            {'robot_description': leader_description},
        ],
        remappings=[
            ('tf', '/tf'),
            ('tf_static', '/tf_static'),
        ],
        condition=IfCondition(spawn_leader),
    )

    leader_joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='leader',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': leader_description},
        ],
        condition=IfCondition(spawn_leader),
    )

    spawn_turtlebot = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        name='spawn_turtlebot4',
        output='screen',
        arguments=[
            '-name', robot_name,
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-topic', 'robot_description',
        ],
    )

    spawn_leader_turtlebot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_leader_turtlebot4',
        output='screen',
        arguments=[
            '-name', leader_name,
            '-x', LaunchConfiguration('leader_x'),
            '-y', LaunchConfiguration('leader_y'),
            '-z', LaunchConfiguration('leader_z'),
            '-Y', LaunchConfiguration('leader_yaw'),
            '-topic', '/leader/robot_description',
        ],
        condition=IfCondition(spawn_leader),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='lidar_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            [
                '/world/', world,
                '/model/', robot_name,
                '/link/rplidar_link/sensor/rplidar/scan'
                '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            ],
        ],
        remappings=[
            (
                [
                    '/world/', world,
                    '/model/', robot_name,
                    '/link/rplidar_link/sensor/rplidar/scan',
                ],
                'scan',
            ),
        ],
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        namespace=namespace,
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            [
                '/world/', world,
                '/model/', robot_name,
                '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image'
                '@sensor_msgs/msg/Image[gz.msgs.Image',
            ],
            [
                '/world/', world,
                '/model/', robot_name,
                '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info'
                '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            ],
        ],
        remappings=[
            (
                [
                    '/world/', world,
                    '/model/', robot_name,
                    '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/image',
                ],
                'oakd/rgb/preview/image_raw',
            ),
            (
                [
                    '/world/', world,
                    '/model/', robot_name,
                    '/link/oakd_rgb_camera_frame/sensor/rgbd_camera/camera_info',
                ],
                'oakd/rgb/preview/camera_info',
            ),
        ],
    )

    leader_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='leader_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
        ],
        remappings=[
            ('/cmd_vel', '/leader/cmd_vel'),
            ('/odom', '/leader/odom'),
        ],
        condition=IfCondition(spawn_leader),
    )

    leader_odom_tf = Node(
        package='sft_hardware_tracker',
        executable='leader_odom_tf_node',
        name='leader_odom_tf_node',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'odom_topic': '/leader/odom'},
            {'parent_frame': ParameterValue([namespace, '/odom'], value_type=str)},
            {'child_frame': 'leader/base_link'},
            {'spawn_x': ParameterValue(LaunchConfiguration('leader_x'), value_type=float)},
            {'spawn_y': ParameterValue(LaunchConfiguration('leader_y'), value_type=float)},
            {'spawn_z': ParameterValue(LaunchConfiguration('leader_z'), value_type=float)},
            {'spawn_yaw': ParameterValue(LaunchConfiguration('leader_yaw'), value_type=float)},
        ],
        condition=IfCondition(spawn_leader),
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        name='joint_state_broadcaster_spawner',
        output='screen',
        parameters=[control_config_file],
        arguments=[
            'joint_state_broadcaster',
            '-c',
            'controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
    )

    diffdrive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=namespace,
        name='diffdrive_controller_spawner',
        output='screen',
        parameters=[control_config_file],
        arguments=[
            'diffdrive_controller',
            '-c',
            'controller_manager',
            '--controller-manager-timeout',
            '60',
        ],
    )

    rplidar_scan_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rplidar_scan_frame_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', [namespace, '/rplidar_link'],
            '--child-frame-id', 'rplidar_link',
        ],
    )

    rplidar_sensor_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rplidar_sensor_frame_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', [namespace, '/rplidar_link'],
            '--child-frame-id', [robot_name, '/rplidar_link/rplidar'],
        ],
    )

    rplidar_sensor_scan_frame_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rplidar_sensor_scan_frame_tf',
        output='screen',
        arguments=[
            '--x', '0',
            '--y', '0',
            '--z', '0',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', [namespace, '/rplidar_link'],
            '--child-frame-id', [robot_name, '/rplidar_link/rplidar/scan'],
        ],
    )

    board_pose = Node(
        package='board_pose_ros',
        executable='board_pose_node',
        name='board_pose_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'image_topic': '/robot_09/oakd/rgb/preview/image_raw',
            'image_type': 'raw',
            'camera_info_topic': '/robot_09/oakd/rgb/preview/camera_info',
            'use_camera_info': True,
            'camera_frame': 'robot_09/oakd_rgb_camera_optical_frame',
            'board_frame': 'board_frame',
            'calibration_file': camera_calibration_file,
            'board_config_file': sim_board_config_file,
            'publish_static_camera_tf': False,
            'publish_debug_image': False,
            'process_every_n': 2,
            'log_pose': True,
            'log_every_n': 20,
            'log_rpy_degrees': False,
        }],
        condition=IfCondition(start_board_pose),
    )

    tracker = Node(
        package='sft_hardware_tracker',
        executable='board_tracker_node',
        name='board_tracker_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_tracker),
    )

    follower = Node(
        package='sft_hardware_tracker',
        executable='recovery_follower_node',
        name='recovery_follower_node',
        output='screen',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(start_follower),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='sft_gazebo_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot_09',
            description='ROS namespace that matches the real TurtleBot hardware topics.',
        ),
        DeclareLaunchArgument(
            'robot_name',
            default_value='turtlebot4',
            description='Gazebo model name for the simulated TurtleBot 4.',
        ),
        DeclareLaunchArgument(
            'world',
            default_value='room',
            choices=['room', 'maze', 'warehouse', 'depot'],
            description='Gazebo world from sft_hardware_tracker/worlds.',
        ),
        DeclareLaunchArgument(
            'turtlebot_model',
            default_value='lite',
            choices=['standard', 'lite'],
            description='TurtleBot 4 model variant.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            choices=['true', 'false'],
            description='Use Gazebo simulation time.',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial robot x position.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial robot y position.'),
        DeclareLaunchArgument('z', default_value='0.05', description='Initial robot z position.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot yaw.'),
        DeclareLaunchArgument(
            'start_board_pose',
            default_value='false',
            choices=['true', 'false'],
            description='Start simulated board pose detection from the ego OAK-D camera.',
        ),
        DeclareLaunchArgument(
            'start_tracker',
            default_value='false',
            choices=['true', 'false'],
            description='Start the board tracker node.',
        ),
        DeclareLaunchArgument(
            'start_follower',
            default_value='false',
            choices=['true', 'false'],
            description='Start the recovery follower node.',
        ),
        DeclareLaunchArgument(
            'spawn_leader',
            default_value='true',
            choices=['true', 'false'],
            description='Spawn a teleop-driven leader TurtleBot carrying the SFT ArUco board.',
        ),
        DeclareLaunchArgument(
            'leader_name',
            default_value='leader_turtlebot4',
            description='Gazebo model name for the leader TurtleBot.',
        ),
        DeclareLaunchArgument('leader_x', default_value='1.5', description='Initial leader robot x position.'),
        DeclareLaunchArgument('leader_y', default_value='0.0', description='Initial leader robot y position.'),
        DeclareLaunchArgument('leader_z', default_value='0.05', description='Initial leader robot z position.'),
        DeclareLaunchArgument('leader_yaw', default_value='0.0', description='Initial leader robot yaw.'),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz with the SFT TurtleBot config.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='RViz config file.',
        ),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', ':'.join(gz_resource_paths)),
        gazebo,
        clock_bridge,
        robot_state_publisher,
        joint_state_publisher,
        leader_state_publisher,
        leader_joint_state_publisher,
        TimerAction(period=5.0, actions=[spawn_turtlebot]),
        TimerAction(period=5.5, actions=[spawn_leader_turtlebot]),
        TimerAction(period=6.0, actions=[lidar_bridge, camera_bridge]),
        TimerAction(period=6.5, actions=[leader_bridge, leader_odom_tf]),
        TimerAction(period=8.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=10.0, actions=[diffdrive_controller_spawner]),
        rplidar_scan_frame_tf,
        rplidar_sensor_frame_tf,
        rplidar_sensor_scan_frame_tf,
        TimerAction(period=8.0, actions=[board_pose]),
        tracker,
        follower,
        rviz_node,
    ])
