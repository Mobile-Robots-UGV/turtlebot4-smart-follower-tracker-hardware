import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def make_slam_include(slam_launch_file, slam_params_file, use_sim_time, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'use_lifecycle_manager': 'false',
            'slam_params_file': slam_params_file,
        }.items(),
        condition=condition,
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('sft_hardware_tracker')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    gazebo_launch_file = os.path.join(pkg_share, 'launch', 'sft_turtlebot_gazebo.launch.py')
    slam_launch_file = os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
    slam_params_default = os.path.join(pkg_share, 'config', 'sft_slam_toolbox.yaml')
    rviz_config_default = os.path.join(pkg_share, 'rviz', 'sft_turtlebot_slam.rviz')

    namespace = LaunchConfiguration('namespace')
    world = LaunchConfiguration('world')
    turtlebot_model = LaunchConfiguration('turtlebot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    start_gazebo = LaunchConfiguration('start_gazebo')
    spawn_leader = LaunchConfiguration('spawn_leader')
    start_board_pose = LaunchConfiguration('start_board_pose')
    start_tracker = LaunchConfiguration('start_tracker')
    start_follower = LaunchConfiguration('start_follower')
    slam_rviz = LaunchConfiguration('slam_rviz')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            'namespace': namespace,
            'world': world,
            'turtlebot_model': turtlebot_model,
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'start_board_pose': start_board_pose,
            'start_tracker': start_tracker,
            'start_follower': start_follower,
            'spawn_leader': spawn_leader,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'yaw': LaunchConfiguration('yaw'),
            'leader_x': LaunchConfiguration('leader_x'),
            'leader_y': LaunchConfiguration('leader_y'),
            'leader_z': LaunchConfiguration('leader_z'),
            'leader_yaw': LaunchConfiguration('leader_yaw'),
        }.items(),
        condition=IfCondition(start_gazebo),
    )

    immediate_slam = make_slam_include(
        slam_launch_file,
        slam_params_file,
        use_sim_time,
        condition=UnlessCondition(start_gazebo),
    )

    delayed_slam = TimerAction(
        period=12.0,
        actions=[
            make_slam_include(
                slam_launch_file,
                slam_params_file,
                use_sim_time,
            )
        ],
        condition=IfCondition(start_gazebo),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='sft_slam_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        additional_env={'LIBGL_DRI3_DISABLE': '1'},
        condition=IfCondition(slam_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot_09',
            description='ROS namespace that matches the real TurtleBot hardware topics.',
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
        DeclareLaunchArgument(
            'start_gazebo',
            default_value='true',
            choices=['true', 'false'],
            description='Start Gazebo TurtleBot bringup before SLAM.',
        ),
        DeclareLaunchArgument(
            'slam_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz with the SLAM config.',
        ),
        DeclareLaunchArgument(
            'spawn_leader',
            default_value='true',
            choices=['true', 'false'],
            description='Spawn a teleop-driven leader TurtleBot carrying the SFT ArUco board.',
        ),
        DeclareLaunchArgument(
            'start_board_pose',
            default_value='true',
            choices=['true', 'false'],
            description='Start simulated board pose detection from the ego OAK-D camera.',
        ),
        DeclareLaunchArgument(
            'start_tracker',
            default_value='true',
            choices=['true', 'false'],
            description='Start the KF/PF board tracker.',
        ),
        DeclareLaunchArgument(
            'start_follower',
            default_value='true',
            choices=['true', 'false'],
            description='Start the recovery follower controller.',
        ),
        DeclareLaunchArgument('x', default_value='0.0', description='Initial robot x position.'),
        DeclareLaunchArgument('y', default_value='0.0', description='Initial robot y position.'),
        DeclareLaunchArgument('z', default_value='0.05', description='Initial robot z position.'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Initial robot yaw.'),
        DeclareLaunchArgument('leader_x', default_value='1.5', description='Initial leader robot x position.'),
        DeclareLaunchArgument('leader_y', default_value='0.0', description='Initial leader robot y position.'),
        DeclareLaunchArgument('leader_z', default_value='0.05', description='Initial leader robot z position.'),
        DeclareLaunchArgument('leader_yaw', default_value='0.0', description='Initial leader robot yaw.'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=slam_params_default,
            description='SLAM Toolbox parameter file.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='RViz config file.',
        ),
        gazebo,
        immediate_slam,
        delayed_slam,
        rviz_node,
    ])
