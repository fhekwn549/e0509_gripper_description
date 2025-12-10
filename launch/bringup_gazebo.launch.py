import os

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('name',  default_value='dsr01',       description='NAME_SPACE'),
        DeclareLaunchArgument('host',  default_value='127.0.0.1',   description='ROBOT_IP'),
        DeclareLaunchArgument('port',  default_value='12345',       description='ROBOT_PORT'),
        DeclareLaunchArgument('mode',  default_value='virtual',     description='OPERATION MODE'),
        DeclareLaunchArgument('model', default_value='e0509',       description='ROBOT_MODEL'),
        DeclareLaunchArgument('color', default_value='white',       description='ROBOT_COLOR'),
        DeclareLaunchArgument('x',     default_value='0',           description='Location x'),
        DeclareLaunchArgument('y',     default_value='0',           description='Location y'),
        DeclareLaunchArgument('z',     default_value='0',           description='Location z'),
        DeclareLaunchArgument('gui',   default_value='false',        description='Start RViz2'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='ROBOT_RT_IP'),
    ]

    # Paths
    pkg_path = get_package_share_directory('e0509_gripper_description')
    xacro_file = os.path.join(pkg_path, 'urdf', 'e0509_with_gripper.urdf.xacro')
    
    # Robot description
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' namespace:=', LaunchConfiguration('name'),
        ' use_gazebo:=true',
        ' host:=', LaunchConfiguration('host'),
        ' port:=', LaunchConfiguration('port'),
        ' rt_host:=', LaunchConfiguration('rt_host'),
        ' mode:=', LaunchConfiguration('mode'),
        ' model:=', LaunchConfiguration('model'),
        ' color:=', LaunchConfiguration('color'),
    ])

    robot_description = {'robot_description': robot_description_content}

    # Run emulator (virtual mode only)
    run_emulator_node = Node(
        package='dsr_bringup2',
        executable='run_emulator',
        namespace=LaunchConfiguration('name'),
        parameters=[
            {'name':    LaunchConfiguration('name')},
            {'rate':    100},
            {'standby': 5000},
            {'command': True},
            {'host':    LaunchConfiguration('host')},
            {'port':    LaunchConfiguration('port')},
            {'mode':    LaunchConfiguration('mode')},
            {'model':   LaunchConfiguration('model')},
            {'gripper': 'none'},
            {'mobile':  'none'},
            {'rt_host': LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('mode'), "' == 'virtual'"])),
        output='screen',
    )

    # Robot State Publisher
    robot_state_pub_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=LaunchConfiguration('name'),
        output='screen',
        parameters=[robot_description],
    )

    # Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf'}.items(),
    )

    # Spawn robot in Gazebo
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=LaunchConfiguration('name'),
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'e0509_gripper',
            '-allow_renaming', 'true',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
        ],
    )

    # Controller spawners
    joint_state_broadcaster_spawner = TimerAction(
        period=2.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        )]
    )

    joint_trajectory_controller_spawner = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['joint_trajectory_controller', '-c', 'controller_manager'],
        )]
    )

    gripper_controller_spawner = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=LaunchConfiguration('name'),
            arguments=['gripper_controller', '-c', 'controller_manager'],
        )]
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('dsr_description2'), 'rviz', 'default.rviz'
    ])
    
    rviz_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='rviz2',
            executable='rviz2',
            namespace=LaunchConfiguration('name'),
            name='rviz2',
            arguments=['-d', rviz_config_file],
            condition=IfCondition(LaunchConfiguration('gui')),
            output='log',
        )]
    )

    return LaunchDescription(ARGUMENTS + [
        run_emulator_node,
        robot_state_pub_node,
        gazebo,
        gz_spawn,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
        gripper_controller_spawner,
        rviz_node,
    ])
