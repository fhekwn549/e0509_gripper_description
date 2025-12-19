#!/usr/bin/env python3
"""
RViz 시각화용 통합 조인트 상태 발행 노드

역할:
    - dynamic_joint_states에서 로봇 팔 조인트 상태 수신
    - gripper/stroke 토픽에서 그리퍼 stroke 값 수신
    - 로봇 팔 + 그리퍼 조인트를 합쳐서 joint_states로 발행
    - RViz에서 전체 로봇 시각화

Note:
    그리퍼 제어는 gripper_service_node.py가 담당합니다.
    이 노드는 시각화만 담당합니다.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from control_msgs.msg import DynamicJointState
from std_msgs.msg import Int32


class GripperJointPublisher(Node):
    def __init__(self):
        super().__init__('gripper_joint_publisher')

        # Publisher for joint states
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.timer = self.create_timer(0.02, self.publish_joint_states)  # 50Hz

        # Robot arm joint names
        self.arm_joint_names = [
            'joint_1', 'joint_2', 'joint_3',
            'joint_4', 'joint_5', 'joint_6'
        ]

        # Gripper joint names
        self.gripper_joint_names = [
            'gripper_rh_r1',
            'gripper_rh_r2',
            'gripper_rh_l1',
            'gripper_rh_l2'
        ]

        # All joint names
        self.all_joint_names = self.arm_joint_names + self.gripper_joint_names

        # Robot arm joint positions (from dynamic_joint_states)
        self.arm_positions = {name: 0.0 for name in self.arm_joint_names}
        self.arm_velocities = {name: 0.0 for name in self.arm_joint_names}

        # Gripper position control
        # stroke: 0 = open, 700 = fully closed (real gripper)
        # joint angle: 0.0 = open, ~1.0 rad = closed
        self.stroke = 0  # Current stroke value (0~700)
        self.target_stroke = 0
        self.stroke_speed = 50  # stroke units per cycle (faster for visualization)

        # Stroke to joint angle conversion
        # stroke 700 → ~1.0 rad
        self.stroke_to_rad = 1.0 / 700.0

        # Dynamic joint states 구독 (로봇 팔 조인트)
        self.dynamic_sub = self.create_subscription(
            DynamicJointState, 'dynamic_joint_states',
            self.dynamic_joint_state_callback, 10)

        # Stroke 토픽 구독 (gripper_service_node가 발행)
        self.stroke_sub = self.create_subscription(
            Int32, 'gripper/stroke', self.stroke_callback, 10)

        self.get_logger().info('========================================')
        self.get_logger().info('Combined Joint Publisher Ready!')
        self.get_logger().info('----------------------------------------')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  dynamic_joint_states - Arm joints')
        self.get_logger().info('  gripper/stroke - Int32 (0~700)')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  joint_states - All joint angles')
        self.get_logger().info('========================================')

    def dynamic_joint_state_callback(self, msg: DynamicJointState):
        """로봇 팔 조인트 상태 수신"""
        for i, name in enumerate(msg.joint_names):
            if name in self.arm_positions and i < len(msg.interface_values):
                interface_val = msg.interface_values[i]
                # position은 보통 첫 번째 인터페이스
                if len(interface_val.values) > 0:
                    self.arm_positions[name] = interface_val.values[0]
                if len(interface_val.values) > 1:
                    self.arm_velocities[name] = interface_val.values[1]

    def stroke_callback(self, msg):
        """토픽으로 stroke 값 수신"""
        stroke = max(0, min(700, msg.data))
        self.target_stroke = stroke
        self.get_logger().debug(f'Received stroke: {stroke}')

    def publish_joint_states(self):
        # Smooth movement towards target for gripper
        if abs(self.stroke - self.target_stroke) > 1:
            if self.stroke < self.target_stroke:
                self.stroke = min(self.stroke + self.stroke_speed, self.target_stroke)
            else:
                self.stroke = max(self.stroke - self.stroke_speed, self.target_stroke)
        else:
            self.stroke = self.target_stroke

        # Convert stroke to joint angle
        gripper_angle = self.stroke * self.stroke_to_rad

        # Build combined joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.all_joint_names

        # Arm positions
        positions = [self.arm_positions[name] for name in self.arm_joint_names]
        velocities = [self.arm_velocities[name] for name in self.arm_joint_names]

        # Gripper positions (all 4 joints same angle)
        positions.extend([gripper_angle] * 4)
        velocities.extend([0.0] * 4)

        msg.position = positions
        msg.velocity = velocities
        msg.effort = [0.0] * len(self.all_joint_names)
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = GripperJointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
