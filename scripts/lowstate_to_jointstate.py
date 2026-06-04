#!/usr/bin/env python3
"""Relay for mapping Unitree Go lowstate motor q to /joint_states.

Map (motor_state index -> URDF joint name):
0 FR_hip_joint
1 FR_thigh_joint
2 FR_calf_joint
3 FL_hip_joint
4 FL_thigh_joint
5 FL_calf_joint
6 RR_hip_joint
7 RR_thigh_joint
8 RR_calf_joint
9 RL_hip_joint
10 RL_thigh_joint
11 RL_calf_joint
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState


MOTOR_STATE_JOINTS = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]


class LowstateToJointState(Node):
    def __init__(self) -> None:
        super().__init__('lowstate_to_joint_state')

        self.declare_parameter('lowstate_topic', '/lf/lowstate')
        self.declare_parameter('joint_state_topic', '/joint_states')

        lowstate_topic = self.get_parameter('lowstate_topic').get_parameter_value().string_value
        joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(JointState, joint_state_topic, 10)
        self.subscription = self.create_subscription(
            LowState,
            lowstate_topic,
            self._on_lowstate,
            10,
        )

        self.get_logger().info(f'Subscribing to {lowstate_topic}, publishing {joint_state_topic}')

    def _on_lowstate(self, msg: LowState) -> None:
        if len(msg.motor_state) < len(MOTOR_STATE_JOINTS):
            self.get_logger().warn(
                f'Received LowState with only {len(msg.motor_state)} motor_state entries, '
                f'expected at least {len(MOTOR_STATE_JOINTS)}'
            )
            return

        state_msg = JointState()
        state_msg.header.stamp = self.get_clock().now().to_msg()
        state_msg.name = MOTOR_STATE_JOINTS

        positions = []
        velocities = []
        efforts = []
        for i in range(len(MOTOR_STATE_JOINTS)):
            motor_state = msg.motor_state[i]
            positions.append(float(motor_state.q))
            velocities.append(float(motor_state.dq))
            efforts.append(float(motor_state.tau_est))

        state_msg.position = positions
        state_msg.velocity = velocities
        state_msg.effort = efforts
        self.publisher.publish(state_msg)


def main() -> None:
    rclpy.init()
    node = LowstateToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
