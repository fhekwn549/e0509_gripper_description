"""
실제 로봇 + Gazebo 디지털 트윈 Launch 파일

실제 로봇을 제어하면서 Gazebo에서 동일하게 시각화합니다.

사용법:
    ros2 launch e0509_gripper_description bringup_real_gazebo.launch.py

    # 또는 에뮬레이터 모드 (테스트용)
    ros2 launch e0509_gripper_description bringup_real_gazebo.launch.py mode:=virtual
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    SetEnvironmentVariable, GroupAction, ExecuteProcess
)
from launch.substitutions import (
    Command, FindExecutable, PathJoinSubstitution,
    LaunchConfiguration, PythonExpression
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ARGUMENTS = [
        # 실제 로봇 설정
        DeclareLaunchArgument('name', default_value='dsr01', description='Real robot namespace'),
        DeclareLaunchArgument('host', default_value='192.168.137.100', description='Robot IP'),
        DeclareLaunchArgument('port', default_value='12345', description='Robot port'),
        DeclareLaunchArgument('mode', default_value='real', description='real or virtual'),
        DeclareLaunchArgument('model', default_value='e0509', description='Robot model'),
        DeclareLaunchArgument('color', default_value='white', description='Robot color'),
        DeclareLaunchArgument('rt_host', default_value='192.168.137.50', description='RT IP'),

        # Gazebo 설정
        DeclareLaunchArgument('gazebo_ns', default_value='gz', description='Gazebo namespace'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
    ]

    # Package paths
    pkg_path = get_package_share_directory('e0509_gripper_description')
    rh_pkg_path = get_package_share_directory('rh_p12_rn_a_description')
    dsr_pkg_path = get_package_share_directory('dsr_description2')

    xacro_file = os.path.join(pkg_path, 'urdf', 'e0509_with_gripper.urdf.xacro')
    controllers_file = os.path.join(pkg_path, 'config', 'gz_controllers.yaml')

    real_ns = LaunchConfiguration('name')
    gazebo_ns = LaunchConfiguration('gazebo_ns')
    mode = LaunchConfiguration('mode')

    # ========== 실제 로봇 관련 ==========

    # 실제 로봇용 robot description
    real_robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' namespace:=', real_ns,
        ' use_gazebo:=false',
        ' host:=', LaunchConfiguration('host'),
        ' port:=', LaunchConfiguration('port'),
        ' rt_host:=', LaunchConfiguration('rt_host'),
        ' mode:=', LaunchConfiguration('mode'),
        ' model:=', LaunchConfiguration('model'),
        ' color:=', LaunchConfiguration('color'),
    ])

    robot_controllers = PathJoinSubstitution([
        FindPackageShare("dsr_controller2"), "config", "dsr_controller2.yaml"
    ])

    # Emulator (virtual mode only)
    run_emulator_node = Node(
        package='dsr_bringup2',
        executable='run_emulator',
        namespace=real_ns,
        parameters=[
            {'name': real_ns},
            {'rate': 100},
            {'standby': 5000},
            {'command': True},
            {'host': LaunchConfiguration('host')},
            {'port': LaunchConfiguration('port')},
            {'mode': mode},
            {'model': LaunchConfiguration('model')},
            {'gripper': 'none'},
            {'mobile': 'none'},
            {'rt_host': LaunchConfiguration('rt_host')},
        ],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'virtual'"])),
        output='screen',
    )

    # Real robot controller manager
    real_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace=real_ns,
        parameters=[{"robot_description": real_robot_description}, robot_controllers],
        output="both",
    )

    # Real robot state publisher
    real_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=real_ns,
        output='screen',
        parameters=[
            {'robot_description': real_robot_description},
            {'use_sim_time': False},
        ],
    )

    # Gripper joint publisher (실제 로봇)
    gripper_joint_pub_node = Node(
        package='e0509_gripper_description',
        executable='gripper_joint_publisher.py',
        name='gripper_joint_publisher',
        namespace=real_ns,
        parameters=[{'use_sim_time': False}],
        output='screen',
    )

    # Gripper service node
    gripper_service_node = Node(
        package='e0509_gripper_description',
        executable='gripper_service_node.py',
        name='gripper_service_node',
        namespace=real_ns,
        parameters=[{'mode': mode}],
        output='screen',
    )

    # Real robot controller spawners
    real_joint_state_broadcaster = Node(
        package="controller_manager",
        namespace=real_ns,
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "controller_manager"],
    )

    real_robot_controller = Node(
        package="controller_manager",
        namespace=real_ns,
        executable="spawner",
        arguments=["dsr_controller2", "-c", "controller_manager"],
    )

    # ========== Gazebo 관련 ==========

    # Gazebo 리소스 경로
    gz_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', ''),
            ':',
            os.path.dirname(rh_pkg_path),
            ':',
            os.path.dirname(dsr_pkg_path),
        ]
    )

    # Gazebo용 robot description
    gazebo_robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' namespace:=', gazebo_ns,
        ' use_gazebo:=true',
    ])

    # Gazebo 실행
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py'
        ]),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf'}.items(),
    )

    # Gazebo robot state publisher
    gazebo_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=gazebo_ns,
        output='screen',
        parameters=[
            {'robot_description': gazebo_robot_description},
            {'use_sim_time': False},
        ],
    )

    # Spawn robot in Gazebo
    gz_spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=gazebo_ns,
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'e0509_gripper_gz',
            '-allow_renaming', 'true',
        ],
    )

    # Gazebo controller spawners
    gz_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        namespace=gazebo_ns,
        arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
    )

    gz_joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=gazebo_ns,
        arguments=['joint_trajectory_controller', '-c', 'controller_manager'],
    )

    gz_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        namespace=gazebo_ns,
        arguments=['gripper_controller', '-c', 'controller_manager'],
    )

    # ========== 브릿지 노드 ==========

    bridge_script = os.path.join(
        get_package_share_directory('e0509_gripper_description'),
        '..', '..', 'lib', 'e0509_gripper_description', 'gazebo_bridge.py'
    )

    gazebo_bridge = ExecuteProcess(
        cmd=['python3', bridge_script, '--real-ns', 'dsr01', '--gazebo-ns', 'gz'],
        output='screen',
    )

    # ========== RViz ==========

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dsr_description2"), "rviz", "default.rviz"
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=real_ns,
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='log',
    )

    # ========== 타이밍 조절 ==========

    # 실제 로봇 (7초 후 시작)
    delayed_real_control = TimerAction(period=7.0, actions=[real_control_node])
    delayed_real_joint_state = TimerAction(period=10.0, actions=[real_joint_state_broadcaster])
    delayed_real_controller = TimerAction(period=12.0, actions=[real_robot_controller])
    delayed_gripper_service = TimerAction(period=14.0, actions=[gripper_service_node])
    delayed_rviz = TimerAction(period=15.0, actions=[rviz_node])

    # Gazebo (즉시 시작, 컨트롤러는 지연)
    delayed_gz_joint_state = TimerAction(period=3.0, actions=[gz_joint_state_broadcaster])
    delayed_gz_traj_controller = TimerAction(period=4.0, actions=[gz_joint_trajectory_controller])
    delayed_gz_gripper = TimerAction(period=5.0, actions=[gz_gripper_controller])

    # 브릿지 (모든 것이 준비된 후 시작)
    delayed_bridge = TimerAction(period=16.0, actions=[gazebo_bridge])

    return LaunchDescription(ARGUMENTS + [
        # 환경 설정
        gz_resource_path,

        # 실제 로봇
        run_emulator_node,
        real_robot_state_pub,
        gripper_joint_pub_node,
        delayed_real_control,
        delayed_real_joint_state,
        delayed_real_controller,
        delayed_gripper_service,
        delayed_rviz,

        # Gazebo
        gazebo,
        gazebo_robot_state_pub,
        gz_spawn,
        delayed_gz_joint_state,
        delayed_gz_traj_controller,
        delayed_gz_gripper,

        # 브릿지
        delayed_bridge,
    ])
