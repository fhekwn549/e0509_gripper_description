"""
RViz + Gazebo 디지털 트윈 Launch 파일

RViz에서 joint_state_publisher_gui로 로봇을 조작하면
Gazebo의 로봇이 동일하게 따라갑니다.

사용법:
    ros2 launch e0509_gripper_description rviz_gazebo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
    SetEnvironmentVariable, ExecuteProcess
)
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ARGUMENTS = [
        DeclareLaunchArgument('rviz_ns', default_value='rviz', description='RViz namespace'),
        DeclareLaunchArgument('gazebo_ns', default_value='gz', description='Gazebo namespace'),
    ]

    # Package paths
    pkg_path = get_package_share_directory('e0509_gripper_description')
    rh_pkg_path = get_package_share_directory('rh_p12_rn_a_description')
    dsr_pkg_path = get_package_share_directory('dsr_description2')

    xacro_file = os.path.join(pkg_path, 'urdf', 'e0509_with_gripper.urdf.xacro')

    rviz_ns = LaunchConfiguration('rviz_ns')
    gazebo_ns = LaunchConfiguration('gazebo_ns')

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

    # ========== RViz 쪽 (조작용) ==========

    # RViz용 robot description (Gazebo 플러그인 없이)
    rviz_robot_description = Command([
        FindExecutable(name='xacro'), ' ', xacro_file,
        ' namespace:=', rviz_ns,
        ' use_gazebo:=false',
    ])

    # Robot State Publisher (RViz용)
    rviz_robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=rviz_ns,
        output='screen',
        parameters=[
            {'robot_description': rviz_robot_description},
            {'use_sim_time': False},
        ],
    )

    # Joint State Publisher GUI (슬라이더로 조작)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=rviz_ns,
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': rviz_robot_description},
        ],
    )

    # RViz
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dsr_description2"), "rviz", "default.rviz"
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        namespace=rviz_ns,
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='log',
    )

    # ========== Gazebo 쪽 (따라가는 쪽) ==========

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

    # Gazebo Robot State Publisher
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
    gz_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=gazebo_ns,
            arguments=['joint_state_broadcaster', '-c', 'controller_manager'],
        )]
    )

    gz_joint_trajectory_controller = TimerAction(
        period=4.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=gazebo_ns,
            arguments=['joint_trajectory_controller', '-c', 'controller_manager'],
        )]
    )

    gz_gripper_controller = TimerAction(
        period=5.0,
        actions=[Node(
            package='controller_manager',
            executable='spawner',
            namespace=gazebo_ns,
            arguments=['gripper_controller', '-c', 'controller_manager'],
        )]
    )

    # ========== 브릿지 노드 ==========
    # RViz의 joint_states → Gazebo의 controller로 전달

    bridge_script = os.path.join(
        get_package_share_directory('e0509_gripper_description'),
        '..', '..', 'lib', 'e0509_gripper_description', 'gazebo_bridge.py'
    )

    gazebo_bridge = ExecuteProcess(
        cmd=['python3', bridge_script, '--real-ns', 'rviz', '--gazebo-ns', 'gz'],
        output='screen',
    )

    return LaunchDescription(ARGUMENTS + [
        # 환경 설정
        gz_resource_path,

        # RViz 쪽
        rviz_robot_state_pub,
        joint_state_publisher_gui,
        rviz_node,

        # Gazebo 쪽
        gazebo,
        gazebo_robot_state_pub,
        gz_spawn,
        gz_joint_state_broadcaster,
        gz_joint_trajectory_controller,
        gz_gripper_controller,

        # 브릿지
        gazebo_bridge,
    ])
