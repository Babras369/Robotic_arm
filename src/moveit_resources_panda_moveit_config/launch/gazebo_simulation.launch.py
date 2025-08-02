import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
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
        name='pause', default_value='false',
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
        launch_arguments={
            'pause': LaunchConfiguration('pause'),
            'use_sim_time': 'true'  # ✅ Ensure Gazebo publishes /clock
        }.items()
    )

    # ✅ Fixed: Changed from panda.urdf to panda.xacro to match RViz
    robot_description = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('moveit_resources_panda_moveit_config'),
            'config', 'panda.xacro'  # ✅ Changed from panda.urdf to panda.xacro
        ])
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': True}  # ✅ Added
        ],
        output='screen'
    )

    robot_spawner_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'panda',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0'],
        parameters=[{'use_sim_time': True}],  # ✅ Added
        output='screen'
    )

    #✅ Add delays to ensure proper sequencing
    load_joint_state_broadcaster = TimerAction(
        period=2.0,  # Wait 2 seconds after launch
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'joint_state_broadcaster'],
                output='screen',
                shell=True  # ✅ Added shell=True for better compatibility
            )
        ]
    )

    load_joint_trajectory_controller = TimerAction(
        period=4.0,  # Wait 4 seconds after launch
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                     'panda_arm_controller'],
                output='screen',
                shell=True  # ✅ Added shell=True
            )
        ]
    )

    load_joint_trajectory_controller_hand = TimerAction(
        period=3.0,  # Wait 3 seconds after launch
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                    'panda_hand_controller'],
                output='screen',
                shell=True  # ✅ Added shell=True
            )
        ]
    )

    # ✅ Alternative approach using event handlers (more reliable)
    # Uncomment this if TimerAction doesn't work well:
    
    # load_joint_state_broadcaster = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )
    
    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         'panda_arm_controller'],
    #     output='screen'
    # )

    # load_joint_trajectory_controller_hand = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         'panda_hand_controller'],
    #     output='screen'
    # )

    # # Add event handler to load controllers after robot spawns
    # controller_manager_timeout = ExecuteProcess(
    #     cmd=['timeout', '10', 'bash', '-c',
    #          'until ros2 service list | grep -q controller_manager; do sleep 0.5; done'],
    #     output='screen'
    # )

    return LaunchDescription([
        declare_pause_arg,
        gazebo_launch,
        robot_state_publisher_node,  # ✅ Keep this - needed for robot spawning
        robot_spawner_node,
        load_joint_state_broadcaster,
        load_joint_trajectory_controller,
        load_joint_trajectory_controller_hand,
        #controller_manager_timeout
    ])

