from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    pkg_share = get_package_share_directory('sft_hardware_tracker')
    config_file = os.path.join(pkg_share, 'config', 'sft_hardware_recovery.yaml')
    rviz_config_file = os.path.join(
        pkg_share,
        'rviz',
        'sft_turtlebot_hardware.rviz',
    )

    start_tracker = LaunchConfiguration('start_tracker')
    start_follower = LaunchConfiguration('start_follower')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    ros_domain_id = LaunchConfiguration('ros_domain_id')
    virtual_robot = LaunchConfiguration('virtual_robot')
    turtlebot_model = LaunchConfiguration('turtlebot_model')
    use_sim_time = LaunchConfiguration('use_sim_time')
    virtual_robot_condition = IfCondition(PythonExpression([
        "'", virtual_robot, "' == 'true' and '", rviz, "' == 'true'"
    ]))

    turtlebot_xacro = PathJoinSubstitution([
        FindPackageShare('turtlebot4_description'),
        'urdf',
        turtlebot_model,
        'turtlebot4.urdf.xacro',
    ])

    robot_description = ParameterValue(
        Command([
            'xacro', ' ', turtlebot_xacro, ' ',
            'gazebo:=ignition', ' ',
            'namespace:=robot_09',
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='robot_09',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'frame_prefix': 'robot_09/'},
            {'robot_description': robot_description},
        ],
        condition=virtual_robot_condition,
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace='robot_09',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=virtual_robot_condition,
    )

    tracker = Node(
        package='sft_hardware_tracker',
        executable='board_tracker_node',
        name='board_tracker_node',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(start_tracker),
    )

    follower = Node(
        package='sft_hardware_tracker',
        executable='recovery_follower_node',
        name='recovery_follower_node',
        output='screen',
        parameters=[config_file],
        condition=IfCondition(start_follower),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='sft_hardware_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ros_domain_id',
            default_value='9',
            description='ROS_DOMAIN_ID for robot 09 hardware.',
        ),
        SetEnvironmentVariable('ROS_DOMAIN_ID', ros_domain_id),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time for virtual visualization nodes.',
        ),
        DeclareLaunchArgument(
            'virtual_robot',
            default_value='true',
            description='Publish a local TurtleBot 4 model for RViz when hardware is not running.',
        ),
        DeclareLaunchArgument(
            'turtlebot_model',
            default_value='lite',
            choices=['standard', 'lite'],
            description='TurtleBot 4 model variant for the local RViz model.',
        ),
        DeclareLaunchArgument(
            'start_tracker',
            default_value='true',
            description='Start the board tracker node.',
        ),
        DeclareLaunchArgument(
            'start_follower',
            default_value='true',
            description='Start the recovery follower node.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Start RViz with the TurtleBot hardware tracking config.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_file,
            description='RViz config file.',
        ),
        robot_state_publisher,
        joint_state_publisher,
        tracker,
        follower,
        rviz_node,
    ])
