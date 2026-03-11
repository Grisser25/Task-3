#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState


class RotorSpinNode(Node):
    def __init__(self):
        super().__init__('rotor_spin_node')

        self.state_sub = self.create_subscription(
            String,
            '/drone_state',
            self.state_callback,
            10
        )

        self.joint_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        self.timer = self.create_timer(0.02, self.publish_rotors)

        self.current_state = 'MONITOR'

        self.rotor_names = [
            'rotor_1_joint',
            'rotor_2_joint',
            'rotor_3_joint',
            'rotor_4_joint'
        ]

        self.rotor_positions = [0.0, 0.0, 0.0, 0.0]
        self.spin_signs = [1.0, -1.0, 1.0, -1.0]
        self.angular_speed = 8.0

        self.get_logger().info('Rotor spin node started')

    def state_callback(self, msg):
        self.current_state = msg.data.strip().upper()

        if self.current_state == 'STABILIZE':
            self.angular_speed = 15.0
        elif self.current_state == 'SCAN':
            self.angular_speed = 25.0
        elif self.current_state == 'REPOSITION':
            self.angular_speed = 40.0
        elif self.current_state == 'DESCEND':
            self.angular_speed = 55.0
        elif self.current_state == 'LANDED':
            self.angular_speed = 0.0
        elif self.current_state == 'MONITOR':
            self.angular_speed = 8.0
        else:
            self.angular_speed = 10.0

    def publish_rotors(self):
        dt = 0.02

        for i in range(4):
            self.rotor_positions[i] += self.spin_signs[i] * self.angular_speed * dt

            if self.rotor_positions[i] > 2.0 * math.pi or self.rotor_positions[i] < -2.0 * math.pi:
                self.rotor_positions[i] = math.fmod(self.rotor_positions[i], 2.0 * math.pi)

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.rotor_names
        msg.position = self.rotor_positions
        msg.velocity = [
            self.spin_signs[0] * self.angular_speed,
            self.spin_signs[1] * self.angular_speed,
            self.spin_signs[2] * self.angular_speed,
            self.spin_signs[3] * self.angular_speed
        ]

        self.joint_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RotorSpinNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
