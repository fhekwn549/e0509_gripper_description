#!/usr/bin/env python3
"""
Gazebo Bridge - 실제 로봇 → Gazebo 동기화

실제 로봇의 joint_states를 받아서 Gazebo 로봇의 controller에 전달합니다.
실제 로봇이 움직이면 Gazebo의 로봇도 동일하게 움직입니다.

사용법:
    ros2 run e0509_gripper_description gazebo_bridge.py \
        --real-ns dsr01 --gazebo-ns gz
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration


class GazeboBridge(Node):
    """실제 로봇 → Gazebo 동기화 브릿지"""

    # 로봇 팔 조인트 이름 (순서 중요)
    ARM_JOINTS = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

    # 그리퍼 조인트 이름
    GRIPPER_JOINTS = ['gripper_rh_r1', 'gripper_rh_r2', 'gripper_rh_l1', 'gripper_rh_l2']

    def __init__(self, real_ns='dsr01', gazebo_ns='gz'):
        super().__init__('gazebo_bridge')

        self.real_ns = real_ns
        self.gazebo_ns = gazebo_ns

        # 현재 조인트 위치 저장
        self.joint_positions = {}

        # 실제 로봇 joint_states 구독
        real_topic = f'/{real_ns}/joint_states'
        self.real_sub = self.create_subscription(
            JointState,
            real_topic,
            self.real_joint_state_callback,
            10
        )

        # Gazebo 로봇 arm trajectory controller 발행
        arm_topic = f'/{gazebo_ns}/joint_trajectory_controller/joint_trajectory'
        self.arm_pub = self.create_publisher(JointTrajectory, arm_topic, 10)

        # Gazebo 로봇 gripper controller 발행
        gripper_topic = f'/{gazebo_ns}/gripper_controller/commands'
        self.gripper_pub = self.create_publisher(Float64MultiArray, gripper_topic, 10)

        # 발행 타이머 (50Hz)
        self.timer = self.create_timer(0.02, self.publish_to_gazebo)

        self.get_logger().info('=' * 60)
        self.get_logger().info('  Gazebo Digital Twin Bridge Started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Real robot namespace: {real_ns}')
        self.get_logger().info(f'  Gazebo namespace: {gazebo_ns}')
        self.get_logger().info(f'  Subscribed to: {real_topic}')
        self.get_logger().info(f'  Publishing to: {arm_topic}')
        self.get_logger().info(f'  Publishing to: {gripper_topic}')
        self.get_logger().info('=' * 60)

        self.count = 0

    def real_joint_state_callback(self, msg: JointState):
        """실제 로봇의 joint_states를 저장"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.joint_positions[name] = msg.position[i]

    def publish_to_gazebo(self):
        """저장된 joint 위치를 Gazebo로 발행"""
        if not self.joint_positions:
            return

        # Arm trajectory 발행
        arm_positions = []
        for joint in self.ARM_JOINTS:
            if joint in self.joint_positions:
                arm_positions.append(self.joint_positions[joint])
            else:
                arm_positions.append(0.0)

        if len(arm_positions) == len(self.ARM_JOINTS):
            traj_msg = JointTrajectory()
            traj_msg.joint_names = self.ARM_JOINTS

            point = JointTrajectoryPoint()
            point.positions = arm_positions
            point.time_from_start = Duration(sec=0, nanosec=50000000)  # 50ms

            traj_msg.points = [point]
            self.arm_pub.publish(traj_msg)

        # Gripper position 발행
        gripper_positions = []
        for joint in self.GRIPPER_JOINTS:
            if joint in self.joint_positions:
                gripper_positions.append(self.joint_positions[joint])
            else:
                gripper_positions.append(0.0)

        if gripper_positions:
            gripper_msg = Float64MultiArray()
            gripper_msg.data = gripper_positions
            self.gripper_pub.publish(gripper_msg)

        self.count += 1
        if self.count % 500 == 0:  # 10초마다 로그
            self.get_logger().info(f'Published {self.count} updates to Gazebo')


def main():
    import argparse
    parser = argparse.ArgumentParser(description='Gazebo Bridge')
    parser.add_argument('--real-ns', default='dsr01', help='Real robot namespace')
    parser.add_argument('--gazebo-ns', default='gz', help='Gazebo robot namespace')
    args = parser.parse_args()

    rclpy.init()
    node = GazeboBridge(real_ns=args.real_ns, gazebo_ns=args.gazebo_ns)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nShutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
