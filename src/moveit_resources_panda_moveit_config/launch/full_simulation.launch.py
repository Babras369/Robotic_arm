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
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ========== ARGUMENTS ==========
    # Gazebo arguments
    declare_pause_arg = DeclareLaunchArgument(
        name='pause', default_value='false',  # Changed to false for better startup
        description='Start Gazebo paused'
    )

    declare_world_arg = DeclareLaunchArgument(
        name='world', default_value='',
        description='World file to load in Gazebo'
    )

    # MoveIt arguments
    tutorial_arg = DeclareLaunchArgument(
        "rviz_tutorial", default_value="False", description="Tutorial flag"
    )

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Database flag"
    )

    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="gazebo_ros2_control/GazeboSystem",
        description="ROS2 control hardware interface type to use for the launch file",
    )

    # ========== GAZEBO SETUP ==========
    # Include the Gazebo launch file with world and ground plane
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch', 'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': LaunchConfiguration('pause'),
            'use_sim_time': 'true',
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )

    # ========== MOVEIT CONFIG ==========
    moveit_config = (
        MoveItConfigsBuilder("moveit_resources_panda")
        .robot_description(
            file_path="config/panda.urdf.xacro",
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .planning_pipelines(
            pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # ========== ROBOT DESCRIPTION ==========
    # Get the URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        PathJoinSubstitution([
            FindPackageShare('moveit_resources_panda_moveit_config'),
            'config', 'panda.urdf.xacro'
        ]),
        ' ros2_control_hardware_type:=', LaunchConfiguration('ros2_control_hardware_type')
    ])

    robot_description = {'robot_description': robot_description_content}

    # ========== ROBOT STATE PUBLISHER ==========
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': True}
        ],
        output='screen'
    )

    # ========== STATIC TF ==========
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "panda_link0"],
        parameters=[{'use_sim_time': True}]
    )

    # ========== SPAWN GROUND PLANE ==========
    spawn_ground_plane = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'ground_plane',
            '-database', 'ground_plane'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ========== ROBOT SPAWNER ==========
    robot_spawner_node = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'panda',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.05',
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0'
        ],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # ========== ROS2 CONTROL CONFIGURATION ==========
    ros2_controllers_path = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    # Load controller configuration
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            ros2_controllers_path,
            {'use_sim_time': True}
        ],
        output="screen",
    )

    # ========== CONTROLLER SPAWNERS ==========
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    panda_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_arm_controller", 
            "--controller-manager", "/controller_manager"
        ],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    panda_hand_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "panda_hand_controller", 
            "--controller-manager", "/controller_manager"
        ],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    # ========== MOVEIT MOVE GROUP ==========
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': True}
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # ========== RVIZ ==========
    tutorial_mode = LaunchConfiguration("rviz_tutorial")
    rviz_base = os.path.join(
        get_package_share_directory("moveit_resources_panda_moveit_config"), "launch"
    )
    rviz_full_config = os.path.join(rviz_base, "moveit.rviz")
    rviz_empty_config = os.path.join(rviz_base, "moveit_empty.rviz")
    
    rviz_node_tutorial = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_empty_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
        condition=IfCondition(tutorial_mode),
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {'use_sim_time': True}
        ],
        condition=UnlessCondition(tutorial_mode),
    )

    # ========== MONGODB ==========
    db_config = LaunchConfiguration("db")
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(db_config),
    )

    # ========== SEQUENCED STARTUP ==========
    # Use event handlers for proper sequencing instead of timers
    
    # Start robot state publisher and static tf immediately after Gazebo
    delayed_robot_state_publisher = TimerAction(
        period=2.0,
        actions=[robot_state_publisher_node, static_tf_node]
    )

    # Spawn ground plane and robot after robot description is ready
    delayed_spawners = TimerAction(
        period=4.0,
        actions=[spawn_ground_plane, robot_spawner_node]
    )

    # Start controller manager after robot is spawned
    delayed_controller_manager = TimerAction(
        period=6.0,
        actions=[controller_manager_node]
    )

    # Start controllers after controller manager is ready
    delayed_joint_state_broadcaster = TimerAction(
        period=8.0,
        actions=[joint_state_broadcaster_spawner]
    )

    delayed_arm_controller = TimerAction(
        period=10.0,
        actions=[panda_arm_controller_spawner]
    )

    delayed_hand_controller = TimerAction(
        period=12.0,
        actions=[panda_hand_controller_spawner]
    )

    # Start MoveIt after all controllers are loaded
    delayed_move_group = TimerAction(
        period=14.0,
        actions=[move_group_node]
    )

    # Start RViz last
    delayed_rviz_tutorial = TimerAction(
        period=16.0,
        actions=[rviz_node_tutorial]
    )

    delayed_rviz = TimerAction(
        period=16.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        # Arguments
        declare_pause_arg,
        declare_world_arg,
        tutorial_arg,
        db_arg,
        ros2_control_hardware_type,
        
        # Start Gazebo first
        gazebo_launch,
        
        # Robot description and transforms
        delayed_robot_state_publisher,
        
        # Spawn entities in Gazebo
        delayed_spawners,
        
        # Start control system
        delayed_controller_manager,
        delayed_joint_state_broadcaster,
        delayed_arm_controller,
        delayed_hand_controller,
        
        # Start MoveIt
        delayed_move_group,
        
        # Start visualization
        delayed_rviz_tutorial,
        delayed_rviz,
        
        # Optional database
        #mongodb_server_node,
    ])