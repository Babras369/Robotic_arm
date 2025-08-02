import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable

def generate_launch_description():

    # Declare the 'pause' argument and set it to true
    declare_pause_arg = DeclareLaunchArgument(
        name='pause', default_value='true',
        description='Start Gazebo paused'
    )

    # Include the Gazebo launch file with pause enabled
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={'pause': LaunchConfiguration('pause')}.items()
    )

    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('moveit_resources_panda_moveit_config'),
            'config', 'panda.urdf'
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    robot_spawner_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'panda',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '1.0' ],
        output='screen'
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_panda_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'panda_arm_controller'],
        output='screen'
    )

    return LaunchDescription([
        declare_pause_arg,
        gazebo_launch,
        robot_state_publisher_node,
        robot_spawner_node,
        load_joint_state_broadcaster,
        load_panda_arm_controller,
    ])
