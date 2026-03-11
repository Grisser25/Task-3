#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, Quaternion
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates, EntityState
from gazebo_msgs.srv import SetEntityState


class GazeboCmdVelBridge(Node):
    def __init__(self):
        super().__init__('gazebo_cmdvel_bridge')

        self.model_name = 'pixhawk450_6depth'
        self.dt = 0.05
        self.floor_z = 0.02
        self.hover_z = 1.0

        self.cmd = Twist()
        self.current_pose = None
        self.ready = False
        self.last_landed = False

        self.pose_initialized = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.current_yaw = 0.0

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(ModelStates, '/model_states', self.model_states_callback, 10)

        self.landing_pub = self.create_publisher(Bool, '/landing_complete', 10)

        self.cli = self.create_client(SetEntityState, '/set_entity_state')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /set_entity_state...')

        self.timer = self.create_timer(self.dt, self.update_model)

        self.get_logger().info(f'Gazebo bridge started for model: {self.model_name}')
        self.get_logger().info(f'Landing threshold floor_z = {self.floor_z:.3f}')
        self.get_logger().info(f'Hover height = {self.hover_z:.3f}')

    def cmd_callback(self, msg):
        self.cmd = msg
        self.get_logger().info(
            f"CMD RX: x={msg.linear.x:.2f}, y={msg.linear.y:.2f}, z={msg.linear.z:.2f}, yaw={msg.angular.z:.2f}",
            throttle_duration_sec=0.5
        )

    def model_states_callback(self, msg):
        if self.model_name in msg.name:
            idx = msg.name.index(self.model_name)
            pose = msg.pose[idx]

            # Initialize once from Gazebo, but force visible hover height
            if not self.pose_initialized:
                self.current_pose = pose
                self.current_x = pose.position.x
                self.current_y = pose.position.y
                self.current_z = self.hover_z
                self.current_yaw = self.yaw_from_quat(pose.orientation)
                self.pose_initialized = True
                self.ready = True

                self.get_logger().info(
                    f'Initial pose set from /model_states: '
                    f'x={self.current_x:.3f}, y={self.current_y:.3f}, z={self.current_z:.3f}'
                )
        else:
            self.get_logger().warn(
                f'Model "{self.model_name}" not found in /model_states. Available: {msg.name}',
                throttle_duration_sec=5.0
            )

    def yaw_from_quat(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def quat_from_yaw(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def publish_landing_complete(self, landed: bool):
        msg = Bool()
        msg.data = landed
        self.landing_pub.publish(msg)

    def set_state_done_callback(self, future):
        try:
            _ = future.result()
            self.get_logger().info(
                "SetEntityState response received",
                throttle_duration_sec=0.5
            )
        except Exception as e:
            self.get_logger().error(f"SetEntityState call failed: {e}")

    def update_model(self):
        if not self.ready:
            return

        yaw = self.current_yaw

        vx_body = self.cmd.linear.x
        vy_body = self.cmd.linear.y
        vz = self.cmd.linear.z
        yaw_rate = self.cmd.angular.z

        vx_world = vx_body * math.cos(yaw) - vy_body * math.sin(yaw)
        vy_world = vx_body * math.sin(yaw) + vy_body * math.cos(yaw)

        new_x = self.current_x + vx_world * self.dt
        new_y = self.current_y + vy_world * self.dt
        new_z = self.current_z + vz * self.dt
        new_yaw = self.current_yaw + yaw_rate * self.dt

        # Hover-hold unless descending
        if vz >= 0.0:
            new_z = self.current_z

        self.get_logger().info(
            f"CURRENT_Z={self.current_z:.3f}, NEW_Z={new_z:.3f}, floor_z={self.floor_z:.3f}, vz={vz:.3f}",
            throttle_duration_sec=0.5
        )

        landed = False
        eps = 1e-3
        if new_z <= self.floor_z + eps:
            new_z = self.floor_z
            landed = True
            vx_world = 0.0
            vy_world = 0.0
            vz = 0.0
            yaw_rate = 0.0

        self.publish_landing_complete(landed)

        if landed != self.last_landed:
            if landed:
                self.get_logger().info(
                    f'LANDING DETECTED at z={new_z:.3f} (floor_z={self.floor_z:.3f})'
                )
            else:
                self.get_logger().info('Landing state cleared')
        self.last_landed = landed

        # Update internal pose first
        self.current_x = new_x
        self.current_y = new_y
        self.current_z = new_z
        self.current_yaw = new_yaw

        # Send pose-only command to Gazebo
        state = EntityState()
        state.name = self.model_name
        state.reference_frame = 'world'

        state.pose.position.x = self.current_x
        state.pose.position.y = self.current_y
        state.pose.position.z = self.current_z
        state.pose.orientation = self.quat_from_yaw(self.current_yaw)

        # Zero twist to avoid physics overriding pose updates
        state.twist.linear.x = 0.0
        state.twist.linear.y = 0.0
        state.twist.linear.z = 0.0
        state.twist.angular.x = 0.0
        state.twist.angular.y = 0.0
        state.twist.angular.z = 0.0

        self.get_logger().info(
            f'cmd=({self.cmd.linear.x:.2f},{self.cmd.linear.y:.2f},{self.cmd.linear.z:.2f}) '
            f'pose=({self.current_x:.2f},{self.current_y:.2f},{self.current_z:.2f}) '
            f'landed={landed}',
            throttle_duration_sec=0.5
        )

        req = SetEntityState.Request()
        req.state = state
        future = self.cli.call_async(req)
        future.add_done_callback(self.set_state_done_callback)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboCmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
